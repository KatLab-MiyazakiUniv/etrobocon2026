/**
 * @file   RouteTypes.h
 * @brief  経路探索で使用する共通データ型
 * @author okuyama0528 yutaro-1214
 */

#ifndef ROUTE_TYPES_H
#define ROUTE_TYPES_H

#include <vector>

/**
 * @brief ロボットの向き
 * @param up    : Yが減る方向
 * @param down  : Yが増える方向
 * @param right : Xが増える方向
 * @param left  : Xが減る方向
 */
enum class Direction { UP = 0, RIGHT, DOWN, LEFT };

/**
 * @brief マップ上の座標
 */
struct Point {
  int x;  // X座標
  int y;  // Y座標
};

/**
 * @brief ゲートの色
 */
enum class GoalColor { RED, BLUE, YELLOW };

/**
 * @brief ゲートの情報
 */
struct Gate {
  GoalColor color;  // ゲートの色
  Point start;      // ゲートの始点
  Point end;        // ゲートの終点
};

/**
 * @brief 経路探索中の状態
 */
struct RouteState {
  int x;                // X座標
  int y;                // y座標
  Direction direction;  // ロボットの向き
};

/**
 * @brief ダイクストラ法の探索結果
 */
struct RouteResult {
  bool found = false;             // 経路が見つかったかどうか
  int cost = 0;                   // 経路のコスト
  std::vector<RouteState> route;  // 経路上のロボットの状態
};

/**
 * @brief ゲート通過用情報
 */
struct GatePass {
  Point entrance;       // ゲートの入り口座標
  Point exit;           // ゲートの出口座標
  Direction direction;  // ゲート通過後のロボットの向き
};

/**
 * @brief ゲート通過を含む経路探索結果
 */
struct GateRouteResult {
  bool found = false;                       // ゲートへの経路が見つかったかどうか
  GoalColor color = GoalColor::RED;         // ゲートの色
  Point entrance = { 0, 0 };                // ゲートの入り口座標
  Point exit = { 0, 0 };                    // ゲートの出口座標
  Direction exitDirection = Direction::UP;  // ゲート通過後のロボットの向き
  int cost = 0;                             // 経路のコスト
  std::vector<RouteState> route;            // 経路上のロボットの状態
};

#endif  // ROUTE_TYPES_H