/**
 * @file   DijkstraRoutePlanner.h
 * @brief  ゲートを考慮したダイクストラ法による経路探索
 * @author okuyama0528 yutaro-1214
 */

#ifndef DIJKSTRA_ROUTE_PLANNER_H
#define DIJKSTRA_ROUTE_PLANNER_H

#include "RouteTypes.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <functional>
#include <queue>
#include <vector>

class DijkstraRoutePlanner {
 public:
  /**
   * @brief コンストラクタ
   * @param _gates ゲートの情報
   */
  explicit DijkstraRoutePlanner(const std::vector<Gate>& _gates);

  /**
   * @brief 指定された1地点まで経路探索する
   * @param startX 開始時点のX座標
   * @param startY 開始時点のY座標
   * @param startDirection 開始時点のロボットが向いている方向
   * @param goal ゴールの座標
   * @param goalDirection ゴール地点でロボットが向く方向
   * @return 経路探索結果
   */
  RouteResult search(int startX, int startY, Direction startDirection, const Point& goal,
                     Direction goalDirection);

 private:
  static constexpr int MAP_MIN = 0;                // ETRallyMapの最小座標値
  static constexpr int MAP_MAX = 10;               // ETRallyMapの最大座標値
  static constexpr int MOVE_STEP = 2;              // 1回の移動量
  static constexpr int GRID_SIZE = 6;              // マップのグリッド数
  static constexpr int DIRECTION_COUNT = 4;        // ロボットの向きの種類数
  static constexpr int STRAIGHT_COST = 1;          // 直進コスト
  static constexpr int TURN_90_COST = 3;           // 90度回頭コスト
  static constexpr int TURN_180_COST = 6;          // 180度回頭コスト
  static constexpr int NEAR_GATE_TURN_COST = 100;  // 危険な回頭のコスト

  std::vector<Gate> gates;  // ゲートの情報

  /**
   * @brief 方向転換に必要なコストを計算する
   */
  int calculateTurnCost(Direction currentDirection, Direction nextDirection) const;

  /**
   * @brief 現在位置と向きを考慮して移動コストを計算する
   */
  int calculateMoveCost(int currentX, int currentY, Direction currentDirection,
                        Direction nextDirection) const;

  /**
   * @brief ロボット後方の危険範囲にゲートの足があるか判定する
   */
  bool isGatePostBehind(int x, int y, Direction direction) const;

  /**
   * @brief 現在地点が外周か判定する
   */
  bool isOuterArea(int x, int y) const;

  /**
   * @brief 指定座標が探索可能か判定する
   */
  bool isValid(int x, int y) const;

  /**
   * @brief ゲートによって移動が妨げられるか判定する
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