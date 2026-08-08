/**
 * @file   GateRoutePlanner.cpp
 * @brief  最新のマップ情報から指定ゲートへの経路を探索するクラス
 */

#include "GateRoutePlanner.h"

#include <climits>

GateRoutePlanner::GateRoutePlanner(const MapData& mapData) : mapData(mapData) {}

GateRouteResult GateRoutePlanner::search(int currentX, int currentY, Direction currentDirection,
                                         GoalColor goalColor)
{
  GateRouteResult result;

  // ==========================================
  // 指定されたゲート情報が存在するか確認
  // ==========================================

  if(!mapData.hasGate(goalColor)) {
    return result;
  }

  // ==========================================
  // 最新のマップ情報でダイクストラ法を生成
  // ==========================================

  DijkstraRoutePlanner planner(mapData.getGates());

  // ==========================================
  // ゲートの2つの通過候補を取得
  // ==========================================

  std::vector<GatePass> passes = mapData.getGatePasses(goalColor);

  if(passes.empty()) {
    return result;
  }

  int bestCost = INT_MAX;

  // ==========================================
  // ゲートの両側を比較
  // ==========================================

  for(const GatePass& pass : passes) {
    /*
     * ダイクストラ法ではすべてのゲートを壁として扱う。
     *
     * 目的ゲートについても入口までは通常の壁として扱い、
     * 入口に到達してから最後にゲートを通過する。
     *
     * goalDirectionには、
     * ゲートをくぐる方向を指定する。
     */
    RouteResult candidate
        = planner.search(currentX, currentY, currentDirection, pass.entrance, pass.direction);

    if(!candidate.found) {
      continue;
    }

    /*
     * candidateの最後では、
     * ゲートを通過する方向を向いている。
     *
     * あとは1区間直進して出口へ行くだけなので、
     * STRAIGHT_COSTを追加する。
     */
    int totalCost = candidate.cost + STRAIGHT_COST;

    // すでにもっと良い経路がある
    if(totalCost >= bestCost) {
      continue;
    }

    bestCost = totalCost;

    // ==========================================
    // ゲート出口まで含めた経路を作成
    // ==========================================

    std::vector<RouteState> fullRoute = candidate.route;

    fullRoute.push_back({ pass.exit.x, pass.exit.y, pass.direction });

    // ==========================================
    // 連続した直進をまとめる
    // ==========================================

    std::vector<RouteState> compressedRoute = compressRoute(fullRoute);

    // ==========================================
    // 結果保存
    // ==========================================

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

  // スタート位置
  compressed.push_back(route.front());

  /*
   * 3点を比較する。
   *
   * previous → current → next
   *
   * currentまでと、
   * currentから先で移動方向が変わるなら
   * currentが曲がり地点。
   */
  for(size_t i = 1; i + 1 < route.size(); ++i) {
    const RouteState& previous = route[i - 1];

    const RouteState& current = route[i];

    const RouteState& next = route[i + 1];

    // ==========================================
    // currentでその場回頭している
    // ==========================================

    bool samePositionAsPrevious = previous.x == current.x && previous.y == current.y;

    if(samePositionAsPrevious && previous.direction != current.direction) {
      compressed.push_back(current);

      continue;
    }

    // ==========================================
    // currentの次で進行方向が変化する
    // ==========================================

    bool directionChangesNext = current.direction != next.direction;

    if(directionChangesNext) {
      compressed.push_back(current);
    }
  }

  // 最終地点
  if(route.size() >= 2) {
    compressed.push_back(route.back());
  }

  return compressed;
}