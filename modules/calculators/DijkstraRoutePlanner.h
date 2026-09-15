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
   *
   * @param gates ゲート情報
   */
  explicit DijkstraRoutePlanner(
      const std::vector<Gate>& gates);

  /**
   * @brief 指定地点までの最小コスト経路を探索する
   *
   * @param startX 開始X座標
   * @param startY 開始Y座標
   * @param startDirection 開始方向
   * @param goal ゴール座標
   * @param goalDirection ゴール時の方向
   *
   * @return 経路探索結果
   */
  RouteResult search(
      int startX,
      int startY,
      Direction startDirection,
      const Point& goal,
      Direction goalDirection);

 private:
  /**
   * @brief 登録されているゲート
   */
  const std::vector<Gate>& gates;

  /**
   * @brief 90度回頭コスト
   */
  static constexpr int TURN_90_COST = 2;

  /**
   * @brief 180度回頭コスト
   */
  static constexpr int TURN_180_COST = 4;

  /**
   * @brief 直進コスト
   */
  static constexpr int STRAIGHT_COST = 1;

  /**
   * @brief 回頭時にゲート足との接触危険がある場合のコスト
   */
  static constexpr int NEAR_GATE_TURN_COST = 100;

  /**
   * @brief 1回の移動量
   */
  static constexpr int MOVE_STEP = 2;

  /**
   * @brief 方向数
   */
  static constexpr int DIRECTION_COUNT = 4;

  /**
   * @brief マップ最小座標
   */
  static constexpr int MAP_MIN = 0;

  /**
   * @brief マップ最大座標
   *
   * 実際のマップサイズに合わせて変更する。
   */
  static constexpr int MAP_MAX = 10;

  /**
   * @brief グリッドサイズ
   */
  static constexpr int GRID_SIZE
      = MAP_MAX / MOVE_STEP + 1;

  /**
   * @brief 回頭中心からロボット最後端までの距離
   *
   * グリッド座標単位。
   *
   * 例:
   * 1グリッド = 122.5mm
   * 回頭中心から後端 = 220mm
   *
   * 220 / 122.5 ≒ 1.8
   *
   * 実機に合わせて調整する。
   */
  static constexpr double TURN_SWEEP_RADIUS = 2.0;

  /**
   * @brief 接触判定の余裕
   *
   * グリッド座標単位。
   */
  static constexpr double TURN_SWEEP_MARGIN = 0.3;

  /**
   * @brief 回頭角度を考慮した回頭コストを取得する
   */
  int calculateTurnCost(
      Direction currentDirection,
      Direction nextDirection) const;

  /**
   * @brief 1移動分のコストを計算する
   */
  int calculateMoveCost(
      int currentX,
      int currentY,
      Direction currentDirection,
      Direction nextDirection) const;

  /**
   * @brief ゲート足付近か判定する
   */
  bool isNearGatePost(
      int x,
      int y) const;

  /**
   * @brief 外周領域か判定する
   */
  bool isOuterArea(
      int x,
      int y) const;

  /**
   * @brief 回頭時にロボット後部が通る範囲に
   *        ゲート足が存在するか判定する
   *
   * @param x 回頭中心X
   * @param y 回頭中心Y
   * @param currentDirection 回頭前方向
   * @param nextDirection 回頭後方向
   *
   * @return 接触の危険がある場合true
   */
  bool isTurnBlockedByGatePost(
      int x,
      int y,
      Direction currentDirection,
      Direction nextDirection) const;

  /**
   * @brief Directionを前方向ベクトルへ変換する
   */
  Point directionToVector(
      Direction direction) const;

  /**
   * @brief 角度を-pi～piへ正規化する
   */
  double normalizeRad(
      double angle) const;

  /**
   * @brief 座標が探索可能範囲か判定する
   */
  bool isValid(
      int x,
      int y) const;

  /**
   * @brief 移動時にゲートを横切るか判定する
   */
  bool isBlockedMove(
      int currentX,
      int currentY,
      int nextX,
      int nextY) const;

  /**
   * @brief 状態を1次元インデックスへ変換する
   */
  int stateToIndex(
      int x,
      int y,
      Direction direction) const;

  /**
   * @brief 1次元インデックスを状態へ戻す
   */
  RouteState indexToState(
      int index) const;
};

#endif