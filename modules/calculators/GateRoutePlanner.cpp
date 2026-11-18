
/**
 * @file   GateRoutePlanner.cpp
 * @brief  最新のマップ情報から指定ゲートへの経路を探索するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "GateRoutePlanner.h"

namespace {

  /**
   * @brief 2地点間の移動方向を取得する
   * @param from 移動開始地点
   * @param to 移動終了地点
   * @return 移動方向
   */
  Direction getMoveDirection(const RouteState& from, const RouteState& to)
  {
    if(to.x < from.x) {
      return Direction::RIGHT;
    }

    if(to.x > from.x) {
      return Direction::LEFT;
    }

    if(to.y < from.y) {
      return Direction::UP;
    }

    return Direction::DOWN;
  }

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
   * 格子座標系
   * UP    : Yが減る
   * RIGHT : Xが減る
   * DOWN  : Yが増える
   * LEFT  : Xが増える
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
    // 現在地点からゲート入口（QR①）までの経路を探索
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

    // ゲート入口（QR①）までの経路
    std::vector<RouteState> fullRoute = candidate.route;

    // ゲート出口（QR②）を追加
    fullRoute.push_back({ pass.exit.x, pass.exit.y, pass.direction });

    /*
     * 通過後の最終停止位置。
     * 外周ゲートの場合は、RouteFollowerが
     * 400mm前進して400mm後退するため、
     * 実際の停止位置はゲート入口（QR①）となる。
     *
     * ここでは経路上の出口（QR②）を保持する。
     * 外周ゲート通過後の現在位置は、
     * EtRallyMotion側で入口座標に更新する。
     */
    Point finalPosition = pass.exit;

    // 外周以外なら次の偶数座標まで進む
    if(!isOuterGrid(pass.exit)) {
      finalPosition = getNextEvenPoint(pass.exit, pass.direction);

      fullRoute.push_back({ finalPosition.x, finalPosition.y, pass.direction });

      // 偶数地点まで追加で直進する分のコスト
      totalCost += STRAIGHT_COST;
    }

    if(totalCost >= bestCost) {
      continue;
    }

    // 経路を圧縮
    std::vector<RouteState> compressedRoute = compressRoute(fullRoute);

    // 探索結果を保存
    result.found = true;
    result.color = goalColor;
    result.entrance = pass.entrance;
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

/**
 * @brief 経路を圧縮する
 *
 * 直線上の中間地点を削除し、以下の地点は保持する。
 * ・始点と終点
 * ・その場で方向転換する地点
 * ・移動方向が変わる地点
 *
 * ダイクストラ法は前進のみを探索するため、
 * 前進とバック走行の切り替え判定は行わない。
 */
std::vector<RouteState> GateRoutePlanner::compressRoute(const std::vector<RouteState>& route) const
{
  if(route.size() <= 2) {
    return route;
  }

  std::vector<RouteState> compressedRoute;
  compressedRoute.push_back(route.front());

  for(size_t i = 1; i < route.size() - 1; ++i) {
    const RouteState& previous = route[i - 1];
    const RouteState& current = route[i];
    const RouteState& next = route[i + 1];

    // その場で方向転換する地点は保持する
    if(previous.x == current.x && previous.y == current.y) {
      if(previous.direction != current.direction) {
        compressedRoute.push_back(current);
      }
      continue;
    }

    // 次の地点と同じ座標の場合も、その場での方向転換として扱う
    if(current.x == next.x && current.y == next.y) {
      if(current.direction != next.direction) {
        compressedRoute.push_back(current);
      }
      continue;
    }

    // 前後の移動方向を取得する
    const Direction moveDirectionBefore = getMoveDirection(previous, current);
    const Direction moveDirectionAfter = getMoveDirection(current, next);

    // 進行方向が変わる地点は保持する
    if(moveDirectionBefore != moveDirectionAfter) {
      compressedRoute.push_back(current);
    }
  }

  // 終点を追加する
  compressedRoute.push_back(route.back());

  return compressedRoute;
}
