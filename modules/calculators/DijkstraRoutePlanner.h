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
enum class Direction {
  NORTH = 0,
  EAST,
  SOUTH,
  WEST
};

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
 * startからendまでの線分をゲートとして扱う。
 *
 * 水平ゲート例:
 * (2,6) ～ (6,6)
 *
 * 垂直ゲート例:
 * (6,2) ～ (6,6)
 */
struct Gate {
  Point start;
  Point end;
};

/**
 * @brief 経路探索中の状態
 *
 * 同じ座標でもロボットの向きが違えば
 * 別の状態として扱う。
 */
struct RouteState {
  int x;
  int y;
  Direction direction;
};

/**
 * @brief ダイクストラ法による経路探索結果
 */
struct RouteResult {
  /**
   * @brief 経路が見つかったか
   */
  bool found = false;

  /**
   * @brief 到達したゴール
   */
  Point goal = {0, 0};

  /**
   * @brief 経路の合計コスト
   */
  int cost = 0;

  /**
   * @brief 開始地点からゴールまでの経路
   */
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
   * @param _gates 通行を遮るゲート一覧
   */
  explicit DijkstraRoutePlanner(
      const std::vector<Gate>& _gates);

  /**
   * @brief 2つのゴール候補から最小コストの経路を探索する
   *
   * @param startX 開始X座標
   * @param startY 開始Y座標
   * @param startDirection 開始時の向き
   * @param goal1 ゴール候補1
   * @param goal2 ゴール候補2
   *
   * @return 経路探索結果
   */
  RouteResult search(
      int startX,
      int startY,
      Direction startDirection,
      const Point& goal1,
      const Point& goal2);

private:
  /**
   * @brief マップ上の最小座標
   */
  static constexpr int MAP_MIN = 0;

  /**
   * @brief マップ上の最大座標
   */
  static constexpr int MAP_MAX = 10;

  /**
   * @brief ノード間の座標間隔
   *
   * 使用する座標:
   *
   * 0, 2, 4, 6, 8, 10
   */
  static constexpr int MOVE_STEP = 2;

  /**
   * @brief X/Y方向のノード数
   *
   * 0～10を2刻みなので6個
   */
  static constexpr int GRID_SIZE = 6;

  /**
   * @brief 方向の数
   */
  static constexpr int DIRECTION_COUNT = 4;

  /**
   * @brief 直進コスト
   */
  static constexpr int STRAIGHT_COST = 1;

  /**
   * @brief 90度回頭コスト
   */
  static constexpr int TURN_90_COST = 3;

  /**
   * @brief 180度回頭コスト
   */
  static constexpr int TURN_180_COST = 10;

  /**
   * @brief ゲート一覧
   */
  std::vector<Gate> gates;

  /**
   * @brief 回頭＋直進に必要なコストを計算する
   */
  int calculateMoveCost(
      Direction currentDirection,
      Direction nextDirection) const;

  /**
   * @brief 指定座標が有効か判定する
   */
  bool isValid(int x, int y) const;

  /**
   * @brief 移動先との間にゲートが存在するか判定する
   */
  bool isBlockedMove(
      int currentX,
      int currentY,
      int nextX,
      int nextY) const;

  /**
   * @brief 状態を配列用インデックスに変換する
   */
  int stateToIndex(
      int x,
      int y,
      Direction direction) const;

  /**
   * @brief 配列用インデックスを状態に戻す
   */
  RouteState indexToState(int index) const;
};

#endif  // DIJKSTRA_ROUTE_PLANNER_H
