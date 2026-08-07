/**
 * @file   DijkstraRoutePlanner.h
 * @brief  回頭コストとゲートを考慮したダイクストラ法による経路探索
 */

#ifndef DIJKSTRA_ROUTE_PLANNER_H
#define DIJKSTRA_ROUTE_PLANNER_H

#include <vector>

/**
 * @brief ロボットの向き
 */
enum class Direction { NORTH = 0, EAST, SOUTH, WEST };

/**
 * @brief マップ上の座標
 */
struct Point {
  int x;
  int y;
};

/**
 * @brief 通行を遮るゲート
 *
 * 水平ゲート:
 *   (1,3) ～ (3,3)
 *
 * 垂直ゲート:
 *   (3,1) ～ (3,3)
 */
struct Gate {
  Point start;
  Point end;
};

/**
 * @brief 経路探索中の状態
 *
 * 同じ座標でも向きが異なれば別状態として扱う
 */
struct RouteState {
  int x;
  int y;
  Direction direction;
};

/**
 * @brief 経路探索結果
 */
struct RouteResult {
  bool found = false;

  Point goal = { 0, 0 };

  int cost = 0;

  std::vector<RouteState> route;
};

/**
 * @brief ダイクストラ法による経路探索クラス
 */
class DijkstraRoutePlanner {
 public:
  /**
   * @brief コンストラクタ
   *
   * @param gates 通行を遮るゲート一覧
   */
  explicit DijkstraRoutePlanner(const std::vector<Gate>& gates);

  /**
   * @brief 2つのゴールのうち最小コストで到達できる方を探索する
   *
   * @param startX 開始X座標
   * @param startY 開始Y座標
   * @param startDirection 開始時の向き
   * @param goal1 ゴール候補1
   * @param goal2 ゴール候補2
   *
   * @return 経路探索結果
   */
  RouteResult search(int startX, int startY, Direction startDirection, const Point& goal1,
                     const Point& goal2);

 private:
  static constexpr int MAP_MIN = 0;
  static constexpr int MAP_MAX = 10;

  static constexpr int MOVE_STEP = 2;

  static constexpr int GRID_SIZE = 6;
  static constexpr int DIRECTION_COUNT = 4;

  static constexpr int STRAIGHT_COST = 1;
  static constexpr int TURN_90_COST = 3;
  static constexpr int TURN_180_COST = 10;

  std::vector<Gate> gates;

  /**
   * @brief 現在の向きから次の向きへの移動コストを計算
   */
  int calculateMoveCost(Direction currentDirection, Direction nextDirection) const;

  /**
   * @brief 座標が走行可能なノードか判定
   */
  bool isValid(int x, int y) const;

  /**
   * @brief 現在地点から次地点への移動がゲートで遮られているか判定
   */
  bool isBlockedMove(int currentX, int currentY, int nextX, int nextY) const;

  /**
   * @brief 状態を配列用インデックスへ変換
   */
  int stateToIndex(int x, int y, Direction direction) const;

  /**
   * @brief 配列用インデックスを状態へ変換
   */
  RouteState indexToState(int index) const;
};

#endif  // DIJKSTRA_ROUTE_PLANNER_H