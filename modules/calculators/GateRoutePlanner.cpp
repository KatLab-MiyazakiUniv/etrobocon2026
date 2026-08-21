/**
 * @file   GateRoutePlanner.cpp
 * @brief  最新のマップ情報から指定ゲートへの経路を探索するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "GateRoutePlanner.h"

namespace {

  /**
   * @brief 指定地点がマップの外周上にあるか判定する
   * @param point 判定する格子座標
   * @return true 外周上、false 外周以外
   */
  bool isOuterGrid(const Point& point)
  {
    constexpr int MIN_GRID = 0;
    constexpr int MAX_GRID = 10;

    return point.x == MIN_GRID || point.x == MAX_GRID || point.y == MIN_GRID || point.y == MAX_GRID;
  }

  /**
   * @brief 進行方向にある次の偶数格子座標を取得する
   *
   * 格子座標系
   * UP    : Yが減る
   * RIGHT : Xが減る
   * DOWN  : Yが増える
   * LEFT  : Xが増える
   *
   * 現在座標が偶数の場合も、その次の偶数座標まで進む。
   *
   * @param current 現在地点
   * @param direction 進行方向
   * @return 次の偶数格子座標
   */
  Point getNextEvenPoint(const Point& current, Direction direction)
  {
    Point target = current;

    switch(direction) {
      case Direction::UP:
        do {
          --target.y;
        } while(target.y % 2 != 0);
        break;

      case Direction::RIGHT:
        do {
          --target.x;
        } while(target.x % 2 != 0);
        break;

      case Direction::DOWN:
        do {
          ++target.y;
        } while(target.y % 2 != 0);
        break;

      case Direction::LEFT:
        do {
          ++target.x;
        } while(target.x % 2 != 0);
        break;
    }

    return target;
  }

}  // namespace

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
    // 現在地点からゲート入口までの経路を探索
    RouteResult candidate
        = planner.search(currentX, currentY, currentDirection, pass.entrance, pass.direction);

    if(!candidate.found) {
      continue;
    }

    // ゲート通過分の直進コストを追加
    int totalCost = candidate.cost + STRAIGHT_COST;

    if(totalCost >= bestCost) {
      continue;
    }

    bestCost = totalCost;

    // =====================================================
    // ゲート入口までの経路
    // =====================================================

    std::vector<RouteState> fullRoute = candidate.route;

    // =====================================================
    // ゲート出口を追加
    // =====================================================

    fullRoute.push_back({ pass.exit.x, pass.exit.y, pass.direction });

    /*
     * 通過後の最終停止位置。
     *
     * 外周に到達した場合はゲート出口のまま。
     * 外周以外の場合は、この後で次の偶数座標へ変更する。
     */
    Point finalPosition = pass.exit;

    // =====================================================
    // 外周以外なら次の偶数座標まで進む
    // =====================================================

    if(!isOuterGrid(pass.exit)) {
      finalPosition = getNextEvenPoint(pass.exit, pass.direction);

      fullRoute.push_back({ finalPosition.x, finalPosition.y, pass.direction });

      // 偶数地点まで追加で直進する分のコスト
      totalCost += STRAIGHT_COST;
    }

    // =====================================================
    // 経路を圧縮
    // =====================================================

    std::vector<RouteState> compressedRoute = compressRoute(fullRoute);

    // =====================================================
    // 探索結果を保存
    // =====================================================

    result.found = true;

    result.color = goalColor;

    result.entrance = pass.entrance;

    /*
     * EtRobocon2026側で
     *
     * currentGridX = routeResult.exit.x;
     * currentGridY = routeResult.exit.y;
     *
     * として次回の探索開始地点に使用しているため、
     * 実際に停止する最終地点をexitとして返す。
     */
    result.exit = finalPosition;

    result.exitDirection = pass.direction;

    result.cost = totalCost;

    result.route = compressedRoute;

    /*
     * 追加直進分まで含めたコストを
     * 次の候補との比較に使用する。
     */
    bestCost = totalCost;
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

    // 同じ位置で向きだけ変わった場合
    bool samePositionAsPrevious = previous.x == current.x && previous.y == current.y;

    if(samePositionAsPrevious && previous.direction != current.direction) {
      compressed.push_back(current);

      continue;
    }

    // 次の地点で進行方向が変わる場合
    bool directionChangesNext = current.direction != next.direction;

    if(directionChangesNext) {
      compressed.push_back(current);
    }
  }

  // 最終地点を追加
  if(route.size() >= 2) {
    compressed.push_back(route.back());
  }

  return compressed;
}