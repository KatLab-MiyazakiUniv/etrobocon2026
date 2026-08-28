/**
 * @file   DijkstraRoutePlanner.h
 * @brief  ゲートを考慮したダイクストラ法による経路探索
 * @author okuyama0528 yutaro-1214
 */

#ifndef DIJKSTRA_ROUTE_PLANNER_H
#define DIJKSTRA_ROUTE_PLANNER_H

#include <algorithm>
#include <climits>
#include <cmath>
#include <functional>
#include <queue>
#include <vector>

#include "RouteTypes.h"

class DijkstraRoutePlanner {
 public:
  /**
   * @brief コンストラクタ
   * @param gates ゲートの情報
   */
  explicit DijkstraRoutePlanner(const std::vector<Gate>& gates);

  /**
   * @brief 指定された1地点まで経路探索する
   * @param startX 開始時点のX座標
   * @param startY 開始時点のY座標
   * @param startDirection 開始時点のロボットが向いてる方向
   * @param goal ゴールの座標
   * @param goalDirection ゴールの座標でロボットが向くべき方向
   * @return 経路探索の結果
   */
  RouteResult search(int startX, int startY, Direction startDirection, const Point& goal,
                     Direction goalDirection);

 private:
  static constexpr int MAP_MIN = 0;    // ETRallyMapの最小座標値
  static constexpr int MAP_MAX = 10;   // ETRallyMapの最大座標値
  static constexpr int MOVE_STEP = 2;  // 1マスの移動の際の座標変化量

  static constexpr int GRID_SIZE
      = 6;  // X・Y方向の移動可能なグリッド地点の数（0～10を2刻みで6地点）

  static constexpr int DIRECTION_COUNT = 4;  // ロボットの向きの種類

  static constexpr int STRAIGHT_COST = 1;        // 直進コスト
  static constexpr int TURN_90_COST = 3;         // 90度回頭コスト
  static constexpr int TURN_180_COST = 6;        // 180度回頭コスト
  static constexpr int NEAR_GATE_TURN_COST = 100;  // ゲート足付近の回頭コスト

  std::vector<Gate> gates;  // ゲートの情報

  /**
   * @brief 現在の方向から次の方向へ旋回する際のコストを計算する
   * @param currentDirection 現在向いている方向
   * @param nextDirection 次に向かう方向
   * @return 旋回に必要なコスト
   */
  int calculateTurnCost(Direction currentDirection, Direction nextDirection) const;

  /**
   * @brief 現在地点を考慮して1回の移動コストを計算する
   * @param currentX 現在地点のX座標
   * @param currentY 現在地点のY座標
   * @param currentDirection 現在向いている方向
   * @param nextDirection 次に向かう方向
   * @return 1回の移動に必要な総コスト
   */
  int calculateMoveCost(int currentX, int currentY, Direction currentDirection,
                        Direction nextDirection) const;

  /**
   * @brief 指定地点がゲートの足付近か判定する
   * @param x X座標
   * @param y Y座標
   * @return 足付近ならtrue
   */
  bool isNearGatePost(int x, int y) const;

  /**
   * @brief 指定地点が外周か判定する
   * @param x X座標
   * @param y Y座標
   * @return 外周ならtrue
   */
  bool isOuterArea(int x, int y) const;

  /**
   * @brief 指定した座標が経路探索可能な範囲内か判定する
   */
  bool isValid(int x, int y) const;

  /**
   * @brief 現在地点から次の地点への移動がゲートによって妨げられているか判定する
   */
  bool isBlockedMove(int currentX, int currentY, int nextX, int nextY) const;

  /**
   * @brief 座標と方向を状態番号へ変換する
   */
  int stateToIndex(int x, int y, Direction direction) const;

  /**
   * @brief 状態番号を座標と方向へ変換する
   */
  RouteState indexToState(int index) const;
};

#endif  // DIJKSTRA_ROUTE_PLANNER_H