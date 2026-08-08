/**
 * @file   RouteTypes.h
 * @brief  経路探索で使用する共通データ型
 */

#ifndef ROUTE_TYPES_H
#define ROUTE_TYPES_H

#include <vector>

/**
 * @brief ロボットの向き
 *
 * UP    : Yが減る方向
 * RIGHT : Xが減る方向
 * DOWN  : Yが増える方向
 * LEFT  : Xが増える方向
 */
enum class Direction { UP = 0, RIGHT, DOWN, LEFT };

/**
 * @brief マップ上の座標
 */
struct Point {
  int x;
  int y;
};

/**
 * @brief ゲートの色
 */
enum class GoalColor { RED, BLUE, YELLOW };

/**
 * @brief ゲート
 */
struct Gate {
  GoalColor color;

  Point start;
  Point end;
};

/**
 * @brief 経路探索中の状態
 */
struct RouteState {
  int x;
  int y;
  Direction direction;
};

/**
 * @brief ダイクストラ法の探索結果
 */
struct RouteResult {
  bool found = false;

  int cost = 0;

  std::vector<RouteState> route;
};

/**
 * @brief ゲート通過用情報
 */
struct GatePass {
  Point entrance;
  Point exit;

  Direction direction;
};

/**
 * @brief ゲート通過を含む経路探索結果
 */
struct GateRouteResult {
  bool found = false;

  GoalColor color = GoalColor::RED;

  Point entrance = { 0, 0 };
  Point exit = { 0, 0 };

  Direction exitDirection = Direction::UP;

  int cost = 0;

  std::vector<RouteState> route;
};

#endif  // ROUTE_TYPES_H