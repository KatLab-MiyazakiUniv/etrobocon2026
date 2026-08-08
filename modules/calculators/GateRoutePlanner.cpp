/**
 * @file   GateRoutePlanner.cpp
 * @brief  指定された色のゲートまでの経路を探索するクラス
 */

#include "GateRoutePlanner.h"

#include <climits>

GateRoutePlanner::GateRoutePlanner(const MapData& mapData)
  : mapData(mapData), planner(mapData.getGates())
{
}

GateRouteResult GateRoutePlanner::search(int currentX, int currentY, Direction currentDirection,
                                         GoalColor goalColor)
{
  GateRouteResult result;

  std::vector<GatePass> passes = mapData.getGatePasses(goalColor);

  int bestCost = INT_MAX;

  for(const GatePass& pass : passes) {
    /*
     * 全ゲートを壁として扱い、
     * 目的ゲートの入口まで探索する。
     *
     * 入口ではゲート通過方向を向く。
     */
    RouteResult candidate
        = planner.search(currentX, currentY, currentDirection, pass.entrance, pass.direction);

    if(!candidate.found) {
      continue;
    }

    /*
     * ゲート通過1区間分。
     *
     * candidateの最後では
     * pass.directionを向いているので
     * 直進コストだけ加算する。
     */
    int totalCost = candidate.cost + STRAIGHT_COST;

    if(totalCost >= bestCost) {
      continue;
    }

    bestCost = totalCost;

    // =========================
    // ゲート出口まで追加
    // =========================

    std::vector<RouteState> fullRoute = candidate.route;

    fullRoute.push_back({ pass.exit.x, pass.exit.y, pass.direction });

    // =========================
    // 直進区間をまとめる
    // =========================

    std::vector<RouteState> compressedRoute = compressRoute(fullRoute);

    // =========================
    // 結果
    // =========================

    result.found = true;
    result.color = goalColor;

    result.entrance = pass.entrance;

    result.exit = pass.exit;

    result.exitDirection = pass.direction;

    result.cost = totalCost;

    result.route = compressedRoute;
  }

  return result;
}

std::vector<RouteState> GateRoutePlanner::compressRoute(const std::vector<RouteState>& route) const
{
  std::vector<RouteState> compressed;

  if(route.empty()) {
    return compressed;
  }

  /*
   * スタート地点は必ず残す
   */
  compressed.push_back(route.front());

  /*
   * currentの次で方向が変わる場合、
   * currentが直進区間の終点になる。
   */
  for(size_t i = 1; i + 1 < route.size(); ++i) {
    const RouteState& current = route[i];

    const RouteState& next = route[i + 1];

    bool positionChanged = current.x != next.x || current.y != next.y;

    bool directionChanged = current.direction != next.direction;

    /*
     * 次の状態で方向が変化する場合
     *
     * currentを曲がり地点として残す。
     */
    if(positionChanged && directionChanged) {
      compressed.push_back(current);
    }

    /*
     * 同じ座標で向きだけ変わる場合も残す。
     *
     * 例:
     * (4,4) DOWN
     * (4,4) RIGHT
     */
    if(!positionChanged && directionChanged) {
      compressed.push_back(current);
    }
  }

  /*
   * 最終地点は必ず残す
   */
  if(route.size() >= 2) {
    compressed.push_back(route.back());
  }

  return compressed;
}