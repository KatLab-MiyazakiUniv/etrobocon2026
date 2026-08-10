/**
 * @file   GateRoutePlanner.h
 * @brief  最新のマップ情報から指定ゲートへの経路を探索するクラス
 * @author okuyama0528 yutaro-1214
 */

#ifndef GATE_ROUTE_PLANNER_H
#define GATE_ROUTE_PLANNER_H

#include <vector>
#include <climits>
#include "DijkstraRoutePlanner.h"
#include "MapData.h"
#include "RouteTypes.h"

class GateRoutePlanner {
 public:
  /**
   * @brief コンストラクタ
   * @param mapData 最新のマップ情報
   */
  explicit GateRoutePlanner(const MapData& mapData);

  /**
   * @brief MapDataから最新のゲート情報を取得
   指定されたゲートを通過する最短経路を探索
    * @param currentX 現在のX座標
    * @param currentY 現在のY座標
    * @param currentDirection 現在のロボットの向き
    * @param goalColor 目標ゲートの色
    * @return ゲートへの経路探索の結果
    */
  GateRouteResult search(int currentX, int currentY, Direction currentDirection,
                         GoalColor goalColor);

 private:
  static constexpr int STRAIGHT_COST = 1;  // 直進のコスト
  const MapData& mapData;                  // 最新のマップ情報

  /**
   * @brief 同じ方向への連続した直進区間をまとめる
   */
  std::vector<RouteState> compressRoute(const std::vector<RouteState>& route) const;
};

#endif  // GATE_ROUTE_PLANNER_H