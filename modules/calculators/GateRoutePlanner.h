/**
 * @file   GateRoutePlanner.h
 * @brief  指定された色のゲートまでの経路を探索するクラス
 */

#ifndef GATE_ROUTE_PLANNER_H
#define GATE_ROUTE_PLANNER_H

#include "DijkstraRoutePlanner.h"
#include "MapData.h"
#include "RouteTypes.h"

class GateRoutePlanner {
 public:
  explicit GateRoutePlanner(const MapData& mapData);

  /**
   * @brief 指定色のゲートを通過する最小コスト経路を探索する
   */
  GateRouteResult search(int currentX, int currentY, Direction currentDirection,
                         GoalColor goalColor);

 private:
  static constexpr int STRAIGHT_COST = 1;

  const MapData& mapData;

  DijkstraRoutePlanner planner;

  /**
   * @brief 同じ方向へ進む連続区間をまとめる
   *
   * 例:
   *
   * (0,0) LEFT
   * (2,0) LEFT
   * (4,0) LEFT
   * (4,2) DOWN
   * (4,4) DOWN
   *
   * ↓
   *
   * (0,0) LEFT
   * (4,0) LEFT
   * (4,4) DOWN
   */
  std::vector<RouteState> compressRoute(const std::vector<RouteState>& route) const;
};

#endif  // GATE_ROUTE_PLANNER_H