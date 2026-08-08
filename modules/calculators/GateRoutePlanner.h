/**
 * @file   GateRoutePlanner.h
 * @brief  最新のマップ情報から指定ゲートへの経路を探索するクラス
 */

#ifndef GATE_ROUTE_PLANNER_H
#define GATE_ROUTE_PLANNER_H

#include <vector>

#include "DijkstraRoutePlanner.h"
#include "MapData.h"
#include "RouteTypes.h"

class GateRoutePlanner {
 public:
  explicit GateRoutePlanner(const MapData& mapData);

  /**
   * @brief 指定色のゲートを通過する最小コスト経路を探索する
   *
   * 探索を行うたびにMapDataから最新のゲート情報を取得する。
   */
  GateRouteResult search(int currentX, int currentY, Direction currentDirection,
                         GoalColor goalColor);

 private:
  static constexpr int STRAIGHT_COST = 1;

  const MapData& mapData;

  /**
   * @brief 同じ方向への連続した直進区間をまとめる
   */
  std::vector<RouteState> compressRoute(const std::vector<RouteState>& route) const;
};

#endif  // GATE_ROUTE_PLANNER_H