/**
 * @file   DijkstraRoutePlanner.cpp
 * @brief  ゲートを考慮したダイクストラ法による経路探索
 * @author okuyama0528 yutaro-1214
 */

#include "DijkstraRoutePlanner.h"

/**
 * @brief ダイクストラ法の優先度付きキューで使用するノード
 */
namespace {
  struct QueueNode {
    int cost;
    int index;
    bool operator>(const QueueNode& other) const { return cost > other.cost; }
  };
}  // namespace

DijkstraRoutePlanner::DijkstraRoutePlanner(const std::vector<Gate>& _gates) : gates(_gates) {}

RouteResult DijkstraRoutePlanner::search(int startX, int startY, Direction startDirection,
                                         const Point& goal, Direction goalDirection)
{
  RouteResult result;

  if(!isValid(startX, startY)) {
    return result;
  }

  if(!isValid(goal.x, goal.y)) {
    return result;
  }

  constexpr int STATE_COUNT = GRID_SIZE * GRID_SIZE * DIRECTION_COUNT;

  std::vector<int> distance(STATE_COUNT, INT_MAX);
  std::vector<int> previous(STATE_COUNT, -1);

  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> queue;

  int startIndex = stateToIndex(startX, startY, startDirection);

  distance[startIndex] = 0;

  queue.push({ 0, startIndex });

  constexpr int DX[DIRECTION_COUNT] = { 0, -MOVE_STEP, 0, MOVE_STEP };
  constexpr int DY[DIRECTION_COUNT] = { -MOVE_STEP, 0, MOVE_STEP, 0 };

  while(!queue.empty()) {
    QueueNode currentNode = queue.top();
    queue.pop();

    if(currentNode.cost != distance[currentNode.index]) {
      continue;
    }

    RouteState current = indexToState(currentNode.index);

    for(int directionValue = 0; directionValue < DIRECTION_COUNT; ++directionValue) {
      Direction nextDirection = static_cast<Direction>(directionValue);

      int nextX = current.x + DX[directionValue];
      int nextY = current.y + DY[directionValue];

      if(!isValid(nextX, nextY)) {
        continue;
      }

      if(isBlockedMove(current.x, current.y, nextX, nextY)) {
        continue;
      }

      int moveCost = calculateMoveCost(current.x, current.y, current.direction, nextDirection);

      int nextCost = currentNode.cost + moveCost;

      int nextIndex = stateToIndex(nextX, nextY, nextDirection);

      if(nextCost < distance[nextIndex]) {
        distance[nextIndex] = nextCost;
        previous[nextIndex] = currentNode.index;

        queue.push({ nextCost, nextIndex });
      }
    }
  }

  // ゴールへの4方向の到着方法を比較
  int bestIndex = -1;
  int bestCost = INT_MAX;

  for(int directionValue = 0; directionValue < DIRECTION_COUNT; ++directionValue) {
    Direction arrivalDirection = static_cast<Direction>(directionValue);

    int index = stateToIndex(goal.x, goal.y, arrivalDirection);

    if(distance[index] == INT_MAX) {
      continue;
    }

    int finalTurnCost = calculateTurnCost(arrivalDirection, goalDirection);

    // 外周以外でロボット後方にゲート足がある場合は、回頭コストを100にする
    if(arrivalDirection != goalDirection && !isOuterArea(goal.x, goal.y)
       && isGatePostBehind(goal.x, goal.y, arrivalDirection)) {
      finalTurnCost = NEAR_GATE_TURN_COST;
    }

    int finalCost = distance[index] + finalTurnCost;

    if(finalCost < bestCost) {
      bestCost = finalCost;
      bestIndex = index;
    }
  }

  if(bestIndex == -1) {
    return result;
  }

  // ゴールからスタートまで経路をたどる
  std::vector<RouteState> route;

  for(int index = bestIndex; index != -1; index = previous[index]) {
    route.push_back(indexToState(index));
  }

  std::reverse(route.begin(), route.end());

  // ゴール地点で最終回頭
  if(route.back().direction != goalDirection) {
    route.push_back({ goal.x, goal.y, goalDirection });
  }

  result.found = true;
  result.cost = bestCost;
  result.route = route;

  return result;
}

int DijkstraRoutePlanner::calculateTurnCost(Direction currentDirection,
                                            Direction nextDirection) const
{
  int current = static_cast<int>(currentDirection);
  int next = static_cast<int>(nextDirection);

  int difference = std::abs(current - next);

  // 回頭角度の差を最小化する
  difference = std::min(difference, DIRECTION_COUNT - difference);

  if(difference == 0) {
    return 0;
  }

  if(difference == 1) {
    return TURN_90_COST;
  }

  return TURN_180_COST;
}

int DijkstraRoutePlanner::calculateMoveCost(int currentX, int currentY, Direction currentDirection,
                                            Direction nextDirection) const
{
  int turnCost = calculateTurnCost(currentDirection, nextDirection);

  // 回頭しない場合は直進コストだけ
  if(turnCost == 0) {
    return STRAIGHT_COST;
  }

  /*
   * 外周以外でロボット後方にゲート足がある場合は、
   * 車体後部が回頭時にゲートへ衝突する可能性があるため
   * 回頭コストをNEAR_GATE_TURN_COSTにする。
   */
  if(!isOuterArea(currentX, currentY) && isGatePostBehind(currentX, currentY, currentDirection)) {
    turnCost = NEAR_GATE_TURN_COST;
  }

  return turnCost + STRAIGHT_COST;
}

bool DijkstraRoutePlanner::isGatePostBehind(int x, int y, Direction direction) const
{
  int frontX = 0;
  int frontY = 0;

  switch(direction) {
    case Direction::UP:
      frontX = 0;
      frontY = -1;
      break;

    case Direction::RIGHT:
      frontX = -1;
      frontY = 0;
      break;

    case Direction::DOWN:
      frontX = 0;
      frontY = 1;
      break;

    case Direction::LEFT:
      frontX = 1;
      frontY = 0;
      break;
  }

  for(const Gate& gate : gates) {
    const Point posts[] = { gate.start, gate.end };

    for(const Point& post : posts) {
      int dx = post.x - x;
      int dy = post.y - y;

      /*
       * ゲート足が1グリッドより遠い場合は
       * 回頭時に衝突する可能性が低いため無視する。
       */
      if(std::abs(dx) > MOVE_STEP || std::abs(dy) > MOVE_STEP) {
        continue;
      }

      int dot = dx * frontX + dy * frontY;

      // ゲートがロボットの後方または後ろ斜めにゲート足がある場合危険と判定する
      if(dot < 0) {
        return true;
      }
    }
  }

  return false;
}

bool DijkstraRoutePlanner::isOuterArea(int x, int y) const
{
  return x == MAP_MIN || x == MAP_MAX || y == MAP_MIN || y == MAP_MAX;
}

bool DijkstraRoutePlanner::isValid(int x, int y) const
{
  if(x < MAP_MIN || x > MAP_MAX || y < MAP_MIN || y > MAP_MAX) {
    return false;
  }

  if(x % MOVE_STEP != 0 || y % MOVE_STEP != 0) {
    return false;
  }

  return true;
}

bool DijkstraRoutePlanner::isBlockedMove(int currentX, int currentY, int nextX, int nextY) const
{
  for(const Gate& gate : gates) {
    // 横向きゲート
    if(currentX == nextX) {
      int middleY = (currentY + nextY) / 2;

      if(gate.start.y == gate.end.y) {
        int minX = std::min(gate.start.x, gate.end.x);
        int maxX = std::max(gate.start.x, gate.end.x);

        if(middleY == gate.start.y && currentX >= minX && currentX <= maxX) {
          return true;
        }
      }
    }

    // 縦向きゲート
    if(currentY == nextY) {
      int middleX = (currentX + nextX) / 2;

      if(gate.start.x == gate.end.x) {
        int minY = std::min(gate.start.y, gate.end.y);
        int maxY = std::max(gate.start.y, gate.end.y);

        if(middleX == gate.start.x && currentY >= minY && currentY <= maxY) {
          return true;
        }
      }
    }
  }

  return false;
}

int DijkstraRoutePlanner::stateToIndex(int x, int y, Direction direction) const
{
  int gridX = x / MOVE_STEP;
  int gridY = y / MOVE_STEP;

  return ((gridY * GRID_SIZE + gridX) * DIRECTION_COUNT) + static_cast<int>(direction);
}

RouteState DijkstraRoutePlanner::indexToState(int index) const
{
  int directionValue = index % DIRECTION_COUNT;

  index /= DIRECTION_COUNT;

  int gridX = index % GRID_SIZE;
  int gridY = index / GRID_SIZE;

  return { gridX * MOVE_STEP, gridY * MOVE_STEP, static_cast<Direction>(directionValue) };
}