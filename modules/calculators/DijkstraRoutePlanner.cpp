/**
 * @file   DijkstraRoutePlanner.cpp
 * @brief  ゲートを考慮したダイクストラ法による経路探索
 */

#include "DijkstraRoutePlanner.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <functional>
#include <queue>

namespace {

  struct QueueNode {
    int cost;
    int index;

    bool operator>(const QueueNode& other) const { return cost > other.cost; }
  };

}  // namespace

DijkstraRoutePlanner::DijkstraRoutePlanner(const std::vector<Gate>& gates) : gates(gates) {}

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

  /*
   * UP    : Y - 2
   * RIGHT : X - 2
   * DOWN  : Y + 2
   * LEFT  : X + 2
   */
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

      int nextCost = currentNode.cost + calculateMoveCost(current.direction, nextDirection);

      int nextIndex = stateToIndex(nextX, nextY, nextDirection);

      if(nextCost < distance[nextIndex]) {
        distance[nextIndex] = nextCost;

        previous[nextIndex] = currentNode.index;

        queue.push({ nextCost, nextIndex });
      }
    }
  }

  // ==========================================
  // ゴールへの4方向の到着方法を比較
  // ==========================================

  int bestIndex = -1;
  int bestCost = INT_MAX;

  for(int directionValue = 0; directionValue < DIRECTION_COUNT; ++directionValue) {
    Direction arrivalDirection = static_cast<Direction>(directionValue);

    int index = stateToIndex(goal.x, goal.y, arrivalDirection);

    if(distance[index] == INT_MAX) {
      continue;
    }

    int finalCost = distance[index] + calculateTurnCost(arrivalDirection, goalDirection);

    if(finalCost < bestCost) {
      bestCost = finalCost;
      bestIndex = index;
    }
  }

  if(bestIndex == -1) {
    return result;
  }

  // ==========================================
  // 経路復元
  // ==========================================

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

  difference = std::min(difference, DIRECTION_COUNT - difference);

  if(difference == 0) {
    return 0;
  }

  if(difference == 1) {
    return TURN_90_COST;
  }

  return TURN_180_COST;
}

int DijkstraRoutePlanner::calculateMoveCost(Direction currentDirection,
                                            Direction nextDirection) const
{
  return calculateTurnCost(currentDirection, nextDirection) + STRAIGHT_COST;
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
    // 上下移動
    if(currentX == nextX) {
      int middleY = (currentY + nextY) / 2;

      // 横向きゲート
      if(gate.start.y == gate.end.y) {
        int minX = std::min(gate.start.x, gate.end.x);

        int maxX = std::max(gate.start.x, gate.end.x);

        if(middleY == gate.start.y && currentX >= minX && currentX <= maxX) {
          return true;
        }
      }
    }

    // 左右移動
    if(currentY == nextY) {
      int middleX = (currentX + nextX) / 2;

      // 縦向きゲート
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