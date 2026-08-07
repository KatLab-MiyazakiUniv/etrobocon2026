/**
 * @file   DijkstraRoutePlanner.cpp
 * @brief  回頭コストとゲートを考慮したダイクストラ法による経路探索
 */

#include "DijkstraRoutePlanner.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <functional>
#include <queue>

namespace {

  /**
   * @brief 優先度付きキューに格納するノード
   */
  struct QueueNode {
    int cost;
    int index;

    bool operator>(const QueueNode& other) const { return cost > other.cost; }
  };

}  // namespace

DijkstraRoutePlanner::DijkstraRoutePlanner(const std::vector<Gate>& gates) : gates(gates) {}

RouteResult DijkstraRoutePlanner::search(int startX, int startY, Direction startDirection,
                                         const Point& goal1, const Point& goal2)
{
  RouteResult result;

  // 開始地点が不正
  if(!isValid(startX, startY)) {
    return result;
  }

  // 両方のゴールが不正
  if(!isValid(goal1.x, goal1.y) && !isValid(goal2.x, goal2.y)) {
    return result;
  }

  constexpr int STATE_COUNT = GRID_SIZE * GRID_SIZE * DIRECTION_COUNT;

  std::vector<int> distance(STATE_COUNT, INT_MAX);

  std::vector<int> previous(STATE_COUNT, -1);

  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> queue;

  int startIndex = stateToIndex(startX, startY, startDirection);

  distance[startIndex] = 0;

  queue.push({ 0, startIndex });

  // NORTH, EAST, SOUTH, WEST
  constexpr int DX[DIRECTION_COUNT] = { 0, MOVE_STEP, 0, -MOVE_STEP };

  constexpr int DY[DIRECTION_COUNT] = { -MOVE_STEP, 0, MOVE_STEP, 0 };

  int goalIndex = -1;

  Point selectedGoal = { 0, 0 };

  while(!queue.empty()) {
    QueueNode currentNode = queue.top();

    queue.pop();

    int currentIndex = currentNode.index;

    int currentCost = currentNode.cost;

    // 古いキュー情報は無視
    if(currentCost != distance[currentIndex]) {
      continue;
    }

    RouteState current = indexToState(currentIndex);

    // ゴール1
    if(current.x == goal1.x && current.y == goal1.y) {
      goalIndex = currentIndex;
      selectedGoal = goal1;

      break;
    }

    // ゴール2
    if(current.x == goal2.x && current.y == goal2.y) {
      goalIndex = currentIndex;
      selectedGoal = goal2;

      break;
    }

    // 4方向を探索
    for(int directionValue = 0; directionValue < DIRECTION_COUNT; ++directionValue) {
      Direction nextDirection = static_cast<Direction>(directionValue);

      int nextX = current.x + DX[directionValue];

      int nextY = current.y + DY[directionValue];

      // マップ外または奇数座標
      if(!isValid(nextX, nextY)) {
        continue;
      }

      // ゲートで遮られている
      if(isBlockedMove(current.x, current.y, nextX, nextY)) {
        continue;
      }

      int moveCost = calculateMoveCost(current.direction, nextDirection);

      int nextCost = currentCost + moveCost;

      int nextIndex = stateToIndex(nextX, nextY, nextDirection);

      // より低コストな経路を発見
      if(nextCost < distance[nextIndex]) {
        distance[nextIndex] = nextCost;

        previous[nextIndex] = currentIndex;

        queue.push({ nextCost, nextIndex });
      }
    }
  }

  // ゴールへ到達できなかった
  if(goalIndex == -1) {
    return result;
  }

  // =========================
  // 経路復元
  // =========================

  std::vector<RouteState> route;

  for(int index = goalIndex; index != -1; index = previous[index]) {
    route.push_back(indexToState(index));
  }

  std::reverse(route.begin(), route.end());

  // =========================
  // 結果
  // =========================

  result.found = true;

  result.goal = selectedGoal;

  result.cost = distance[goalIndex];

  result.route = route;

  return result;
}

int DijkstraRoutePlanner::calculateMoveCost(Direction currentDirection,
                                            Direction nextDirection) const
{
  int current = static_cast<int>(currentDirection);

  int next = static_cast<int>(nextDirection);

  int difference = std::abs(current - next);

  // NORTH(0) と WEST(3) は90度差なので補正
  difference = std::min(difference, DIRECTION_COUNT - difference);

  // 直進
  if(difference == 0) {
    return STRAIGHT_COST;
  }

  // 90度回頭 + 直進
  if(difference == 1) {
    return TURN_90_COST + STRAIGHT_COST;
  }

  // 180度回頭 + 直進
  return TURN_180_COST + STRAIGHT_COST;
}

bool DijkstraRoutePlanner::isValid(int x, int y) const
{
  // マップ範囲
  if(x < MAP_MIN || x > MAP_MAX || y < MAP_MIN || y > MAP_MAX) {
    return false;
  }

  // 偶数座標のみ走行
  if(x % MOVE_STEP != 0 || y % MOVE_STEP != 0) {
    return false;
  }

  return true;
}

bool DijkstraRoutePlanner::isBlockedMove(int currentX, int currentY, int nextX, int nextY) const
{
  for(const Gate& gate : gates) {
    // =========================
    // 上下方向への移動
    // =========================

    if(currentX == nextX) {
      // 移動区間の中央
      int middleY = (currentY + nextY) / 2;

      // 横向きゲートのみ対象
      if(gate.start.y == gate.end.y) {
        int gateY = gate.start.y;

        int gateMinX = std::min(gate.start.x, gate.end.x);

        int gateMaxX = std::max(gate.start.x, gate.end.x);

        // 移動線とゲートが交差
        if(middleY == gateY && currentX >= gateMinX && currentX <= gateMaxX) {
          return true;
        }
      }
    }

    // =========================
    // 左右方向への移動
    // =========================

    if(currentY == nextY) {
      // 移動区間の中央
      int middleX = (currentX + nextX) / 2;

      // 縦向きゲートのみ対象
      if(gate.start.x == gate.end.x) {
        int gateX = gate.start.x;

        int gateMinY = std::min(gate.start.y, gate.end.y);

        int gateMaxY = std::max(gate.start.y, gate.end.y);

        // 移動線とゲートが交差
        if(middleX == gateX && currentY >= gateMinY && currentY <= gateMaxY) {
          return true;
        }
      }
    }
  }

  return false;
}

int DijkstraRoutePlanner::stateToIndex(int x, int y, Direction direction) const
{
  // 実座標
  // 0,2,4,6,8,10
  //
  // ↓
  //
  // 内部座標
  // 0,1,2,3,4,5

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