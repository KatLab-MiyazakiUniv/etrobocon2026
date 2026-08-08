/**
 * @file   DijkstraRoutePlanner.h
 * @brief  ゲートを考慮したダイクストラ法による経路探索
 */

#ifndef DIJKSTRA_ROUTE_PLANNER_H
#define DIJKSTRA_ROUTE_PLANNER_H

#include <vector>

#include "RouteTypes.h"

class DijkstraRoutePlanner {
 public:
  explicit DijkstraRoutePlanner(const std::vector<Gate>& gates);

  /**
   * @brief 指定された1地点まで経路探索する
   *
   * @param startX 開始X
   * @param startY 開始Y
   * @param startDirection 開始方向
   * @param goal ゴール座標
   * @param goalDirection ゴール地点で向いてほしい方向
   */
  RouteResult search(int startX, int startY, Direction startDirection, const Point& goal,
                     Direction goalDirection);

 private:
  static constexpr int MAP_MIN = 0;
  static constexpr int MAP_MAX = 10;

  static constexpr int MOVE_STEP = 2;

  static constexpr int GRID_SIZE = 6;
  static constexpr int DIRECTION_COUNT = 4;

  static constexpr int STRAIGHT_COST = 1;
  static constexpr int TURN_90_COST = 3;
  static constexpr int TURN_180_COST = 6;

  std::vector<Gate> gates;

  int calculateTurnCost(Direction currentDirection, Direction nextDirection) const;

  int calculateMoveCost(Direction currentDirection, Direction nextDirection) const;

  bool isValid(int x, int y) const;

  bool isBlockedMove(int currentX, int currentY, int nextX, int nextY) const;

  int stateToIndex(int x, int y, Direction direction) const;

  RouteState indexToState(int index) const;
};

#endif  // DIJKSTRA_ROUTE_PLANNER_H