/**
 * @file   GateRoutePlanner.cpp
 * @brief  最新のマップ情報から指定ゲートへの経路を探索するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "GateRoutePlanner.h"

GateRoutePlanner::GateRoutePlanner(const MapData& mapData) : mapData(mapData) {}

GateRouteResult GateRoutePlanner::search(int currentX, int currentY, Direction currentDirection,
                                         GoalColor goalColor)
{
  GateRouteResult result;

  // 指定されたゲートがマップ上に存在しない場合は探索せずに終了する
  if(!mapData.hasGate(goalColor)) {
    return result;
  }

  // 最新のマップ情報でダイクストラ法を生成
  DijkstraRoutePlanner planner(mapData.getGates());

  // ゲートの2つの通過候補を取得
  std::vector<GatePass> passes = mapData.getGatePasses(goalColor);

  if(passes.empty()) {
    return result;
  }

  int bestCost = INT_MAX;

  for(const GatePass& pass : passes) {
    RouteResult candidate
        = planner.search(currentX, currentY, currentDirection, pass.entrance, pass.direction);

    if(!candidate.found) {
      continue;
    }

    int totalCost = candidate.cost + STRAIGHT_COST;

    if(totalCost >= bestCost) {
      continue;
    }

    bestCost = totalCost;

    std::vector<RouteState> fullRoute = candidate.route;

    fullRoute.push_back({ pass.exit.x, pass.exit.y, pass.direction });

    std::vector<RouteState> compressedRoute = compressRoute(fullRoute);

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

  // 最初の地点を追加
  compressed.push_back(route.front());

  // robotの向きが変わる地点を追加する
  for(size_t i = 1; i + 1 < route.size(); ++i) {
    const RouteState& previous = route[i - 1];
    const RouteState& current = route[i];
    const RouteState& next = route[i + 1];

    bool samePositionAsPrevious = previous.x == current.x && previous.y == current.y;

    if(samePositionAsPrevious && previous.direction != current.direction) {
      compressed.push_back(current);
      continue;
    }

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