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
#include "AngleNormalizer.h"
#include "Logger.h"

class DijkstraRoutePlanner {
 public:
  /**
   * @brief コンストラクタ
   * @param gates ゲート情報
   */
  explicit DijkstraRoutePlanner(const std::vector<Gate>& gates);

  /**
   * @brief 指定地点までの最小コスト経路を探索する
   * @param startX 開始X座標
   * @param startY 開始Y座標
   * @param startDirection 開始方向
   * @param goal ゴール座標
   * @param goalDirection ゴール時の方向
   * @return 経路探索結果
   */
  RouteResult search(int startX, int startY, Direction startDirection, const Point& goal,
                     Direction goalDirection);

 private:
  const std::vector<Gate>& gates;                  // 登録されているゲート
  static constexpr int TURN_90_COST = 2;           // 90度回頭コスト
  static constexpr int TURN_180_COST = 4;          // 180度回頭コスト
  static constexpr int STRAIGHT_COST = 1;          // 直進コスト
  static constexpr int NEAR_GATE_TURN_COST = 100;  // 回頭時にゲート足との接触危険がある場合のコスト
  static constexpr int MOVE_STEP = 2;              // 1回の移動量
  static constexpr int DIRECTION_COUNT = 4;        // 方向数
  static constexpr int MAP_MIN = 0;                // マップ最小座標
  static constexpr int MAP_MAX = 10;               // マップ最大座標
  static constexpr int GRID_SIZE = MAP_MAX / MOVE_STEP + 1;  // グリッドサイズ
  static constexpr double TURN_SWEEP_RADIUS = 2.0;           // 回頭中心からロボット最後端までの距離
  static constexpr double TURN_SWEEP_MARGIN = 0.3;           // 接触判定の余裕

  /**
   * @brief 回頭角度を考慮した回頭コストを取得する
   * @param currentDirection 現在方向
   * @param nextDirection 次の方向
   * @return 回頭コスト
   */
  int calculateTurnCost(Direction currentDirection, Direction nextDirection) const;

  /**
   * @brief 1移動分のコストを計算する
   * @param currentX 現在X座標
   * @param currentY 現在Y座標
   * @param currentDirection 現在方向
   * @param nextDirection 次の方向
   * @return 移動コスト
   */
  int calculateMoveCost(int currentX, int currentY, Direction currentDirection,
                        Direction nextDirection) const;

  /**
   * @brief ゲート足付近か判定する
   * @param x 判定するX座標
   * @param y 判定するY座標
   * @return ゲート足付近の場合true
   */
  bool isNearGatePost(int x, int y) const;

  /**
   * @brief 外周領域か判定する
   * @param x 判定するX座標
   * @param y 判定するY座標
   * @return 外周領域の場合true
   */
  bool isOuterArea(int x, int y) const;

  /**
   * @brief 回頭時にロボット後部が通る範囲に
   *        ゲート足が存在するか判定する
   * @param x 回頭中心X
   * @param y 回頭中心Y
   * @param currentDirection 回頭前方向
   * @param nextDirection 回頭後方向
   * @return 接触の危険がある場合true
   */
  bool isTurnBlockedByGatePost(int x, int y, Direction currentDirection,
                               Direction nextDirection) const;

  /**
   * @brief Directionを前方向ベクトルへ変換する
   * @param direction 方向
   * @return 前方向ベクトル
   */
  Point directionToVector(Direction direction) const;

  /**
   * @brief 座標が探索可能範囲か判定する
   * @param x 判定するX座標
   * @param y 判定するY座標
   * @return 探索可能範囲の場合true
   */
  bool isValid(int x, int y) const;

  /**
   * @brief 移動時にゲートを横切るか判定する
   * @param currentX 現在X座標
   * @param currentY 現在Y座標
   * @param nextX 次のX座標
   * @param nextY 次のY座標
   * @return 移動時にゲートを横切る場合true
   */
  bool isBlockedMove(int currentX, int currentY, int nextX, int nextY) const;

  /**
   * @brief 状態を1次元インデックスへ変換する
   * @param x X座標
   * @param y Y座標
   * @param direction 方向
   * @return 1次元インデックス
   */
  int stateToIndex(int x, int y, Direction direction) const;

  /**
   * @brief 1次元インデックスを状態へ戻す
   * @param index 1次元インデックス
   * @return 状態
   */
  RouteState indexToState(int index) const;
};

#endif