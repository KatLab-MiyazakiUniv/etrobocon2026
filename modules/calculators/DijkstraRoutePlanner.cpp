/**
 * @file   DijkstraRoutePlanner.cpp
 * @brief  回頭コストとゲートを考慮したダイクストラ法による経路探索
 */

#include "DijkstraRoutePlanner.h"

#include <algorithm>
#include <climits>
#include <functional>
#include <queue>
#include <vector>

namespace {

/**
 * @brief 優先度付きキューに格納するノード
 */
struct QueueNode {
  int cost;
  int index;

  /**
   * @brief コスト比較
   *
   * priority_queueで最小コストを
   * 取り出すために使用する。
   */
  bool operator>(const QueueNode& other) const
  {
    return cost > other.cost;
  }
};

}  // namespace

DijkstraRoutePlanner::DijkstraRoutePlanner(
    const std::vector<Gate>& _gates)
    : gates(_gates)
{
}

/**
 * @brief 2つのゴール候補から最小コストの経路を探索
 */
RouteResult DijkstraRoutePlanner::search(
    int startX,
    int startY,
    Direction startDirection,
    const Point& goal1,
    const Point& goal2)
{
  RouteResult result;

  /*
   * =========================
   * 入力チェック
   * =========================
   */

  // 開始地点が不正
  if(!isValid(startX, startY)) {
    return result;
  }

  // 開始方向が不正
  const int directionValue =
      static_cast<int>(startDirection);

  if(directionValue < 0
     || directionValue >= DIRECTION_COUNT) {
    return result;
  }

  // 両方のゴールが不正
  if(!isValid(goal1.x, goal1.y)
     && !isValid(goal2.x, goal2.y)) {
    return result;
  }

  /*
   * =========================
   * 配列初期化
   * =========================
   */

  /*
   * 状態数
   *
   * 6 × 6 × 4 = 144
   *
   * 1つの座標に対して
   * NORTH/EAST/SOUTH/WEST
   * の4状態が存在する。
   */
  constexpr int STATE_COUNT =
      GRID_SIZE
      * GRID_SIZE
      * DIRECTION_COUNT;

  /*
   * 各状態までの最小コスト
   */
  std::vector<int> distance(
      STATE_COUNT,
      INT_MAX);

  /*
   * 経路復元用
   *
   * previous[nextIndex]
   * = nextIndexへ来る直前の状態
   */
  std::vector<int> previous(
      STATE_COUNT,
      -1);

  /*
   * =========================
   * 優先度付きキュー
   * =========================
   */

  std::priority_queue<
      QueueNode,
      std::vector<QueueNode>,
      std::greater<QueueNode>>
      queue;

  /*
   * 開始状態をインデックスへ変換
   */
  const int startIndex =
      stateToIndex(
          startX,
          startY,
          startDirection);

  /*
   * 開始地点のコストは0
   */
  distance[startIndex] = 0;

  queue.push({
      0,
      startIndex
  });

  /*
   * =========================
   * 移動方向
   * =========================
   *
   * NORTH:
   *   y - 2
   *
   * EAST:
   *   x + 2
   *
   * SOUTH:
   *   y + 2
   *
   * WEST:
   *   x - 2
   */

  constexpr int DX[DIRECTION_COUNT] = {
      0,
      MOVE_STEP,
      0,
      -MOVE_STEP
  };

  constexpr int DY[DIRECTION_COUNT] = {
      -MOVE_STEP,
      0,
      MOVE_STEP,
      0
  };

  /*
   * ゴール状態
   */
  int goalIndex = -1;

  /*
   * 実際に選ばれたゴール
   */
  Point selectedGoal = {0, 0};

  /*
   * =========================
   * Dijkstra探索
   * =========================
   */

  while(!queue.empty()) {

    /*
     * 最小コストの状態を取得
     */
    const QueueNode currentNode =
        queue.top();

    queue.pop();

    const int currentIndex =
        currentNode.index;

    const int currentCost =
        currentNode.cost;

    /*
     * =========================
     * 古いキュー情報を無視
     * =========================
     *
     * 同じ状態が複数回queueに
     * 入ることがあるため、
     * 現在の最小コストと一致しない
     * 古い情報は無視する。
     */
    if(currentCost
       != distance[currentIndex]) {

      continue;
    }

    /*
     * インデックスを状態へ戻す
     */
    const RouteState current =
        indexToState(currentIndex);

    /*
     * =========================
     * ゴール判定
     * =========================
     */

    if(current.x == goal1.x
       && current.y == goal1.y) {

      goalIndex = currentIndex;

      selectedGoal = goal1;

      break;
    }

    if(current.x == goal2.x
       && current.y == goal2.y) {

      goalIndex = currentIndex;

      selectedGoal = goal2;

      break;
    }

    /*
     * =========================
     * 4方向を探索
     * =========================
     */

    for(int direction = 0;
        direction < DIRECTION_COUNT;
        ++direction) {

      /*
       * 次に進む方向
       */
      const Direction nextDirection =
          static_cast<Direction>(
              direction);

      /*
       * 次の座標
       */
      const int nextX =
          current.x + DX[direction];

      const int nextY =
          current.y + DY[direction];

      /*
       * =========================
       * 座標チェック
       * =========================
       */

      if(!isValid(nextX, nextY)) {
        continue;
      }

      /*
       * =========================
       * ゲートチェック
       * =========================
       */

      if(isBlockedMove(
             current.x,
             current.y,
             nextX,
             nextY)) {

        continue;
      }

      /*
       * =========================
       * 移動コスト計算
       * =========================
       */

      const int moveCost =
          calculateMoveCost(
              current.direction,
              nextDirection);

      const int nextCost =
          currentCost + moveCost;

      /*
       * 次状態をインデックスへ変換
       */
      const int nextIndex =
          stateToIndex(
              nextX,
              nextY,
              nextDirection);

      /*
       * =========================
       * 最短経路更新
       * =========================
       */

      if(nextCost
         < distance[nextIndex]) {

        /*
         * 最小コストを更新
         */
        distance[nextIndex] =
            nextCost;

        /*
         * 直前の状態を保存
         */
        previous[nextIndex] =
            currentIndex;

        /*
         * queueへ追加
         */
        queue.push({
            nextCost,
            nextIndex
        });
      }
    }
  }

  /*
   * =========================
   * 到達できなかった場合
   * =========================
   */

  if(goalIndex == -1) {
    return result;
  }

  /*
   * =========================
   * 経路復元
   * =========================
   */

  std::vector<RouteState> route;

  /*
   * ゴールからpreviousを辿って
   * 開始地点まで戻る。
   */
  for(int index = goalIndex;
      index != -1;
      index = previous[index]) {

    route.push_back(
        indexToState(index));
  }

  /*
   * 現在は
   *
   * ゴール → 開始
   *
   * の順になっているので
   * 逆順にする。
   */
  std::reverse(
      route.begin(),
      route.end());

  /*
   * =========================
   * 結果を設定
   * =========================
   */

  result.found = true;

  result.goal = selectedGoal;

  result.cost =
      distance[goalIndex];

  result.route =
      route;

  return result;
}

/**
 * @brief 移動コストを計算する
 */
int DijkstraRoutePlanner::calculateMoveCost(
    Direction currentDirection,
    Direction nextDirection) const
{
  const int current =
      static_cast<int>(
          currentDirection);

  const int next =
      static_cast<int>(
          nextDirection);

  /*
   * 向きの差
   */
  int difference =
      std::abs(current - next);

  /*
   * NORTH(0) と WEST(3)
   *
   * 数値上では3離れているが、
   * 実際には90度差。
   *
   * そのため4との差を取る。
   */
  difference =
      std::min(
          difference,
          DIRECTION_COUNT - difference);

  /*
   * =========================
   * 直進
   * =========================
   */
  if(difference == 0) {
    return STRAIGHT_COST;
  }

  /*
   * =========================
   * 90度回頭
   * =========================
   */
  if(difference == 1) {
    return
        TURN_90_COST
        + STRAIGHT_COST;
  }

  /*
   * =========================
   * 180度回頭
   * =========================
   */
  return
      TURN_180_COST
      + STRAIGHT_COST;
}

/**
 * @brief 座標が有効か判定する
 */
bool DijkstraRoutePlanner::isValid(
    int x,
    int y) const
{
  /*
   * マップ範囲
   */
  if(x < MAP_MIN
     || x > MAP_MAX
     || y < MAP_MIN
     || y > MAP_MAX) {

    return false;
  }

  /*
   * 2刻みの座標のみ有効
   *
   * 例:
   *
   * OK:
   * 0,2,4,6,8,10
   *
   * NG:
   * 1,3,5,7,9
   */
  if(x % MOVE_STEP != 0
     || y % MOVE_STEP != 0) {

    return false;
  }

  return true;
}

/**
 * @brief ゲートによる通行禁止を判定する
 */
bool DijkstraRoutePlanner::isBlockedMove(
    int currentX,
    int currentY,
    int nextX,
    int nextY) const
{
  /*
   * 全ゲートを確認
   */
  for(const Gate& gate : gates) {

    /*
     * =========================
     * 上下方向への移動
     * =========================
     *
     * X座標が同じなら
     * 上下方向の移動。
     */
    if(currentX == nextX) {

      /*
       * 移動区間の中央Y座標
       *
       * 例:
       *
       * y=2 → y=4
       *
       * middleY = 3
       */
      const int middleY =
          (currentY + nextY) / 2;

      /*
       * ゲートが水平なら
       * 上下移動を遮る可能性がある。
       */
      if(gate.start.y
         == gate.end.y) {

        const int gateY =
            gate.start.y;

        const int gateMinX =
            std::min(
                gate.start.x,
                gate.end.x);

        const int gateMaxX =
            std::max(
                gate.start.x,
                gate.end.x);

        /*
         * 移動区間とゲートが交差
         */
        if(middleY == gateY
           && currentX >= gateMinX
           && currentX <= gateMaxX) {

          return true;
        }
      }
    }

    /*
     * =========================
     * 左右方向への移動
     * =========================
     *
     * Y座標が同じなら
     * 左右方向の移動。
     */
    if(currentY == nextY) {

      /*
       * 移動区間の中央X座標
       */
      const int middleX =
          (currentX + nextX) / 2;

      /*
       * ゲートが垂直なら
       * 左右移動を遮る可能性がある。
       */
      if(gate.start.x
         == gate.end.x) {

        const int gateX =
            gate.start.x;

        const int gateMinY =
            std::min(
                gate.start.y,
                gate.end.y);

        const int gateMaxY =
            std::max(
                gate.start.y,
                gate.end.y);

        /*
         * 移動区間とゲートが交差
         */
        if(middleX == gateX
           && currentY >= gateMinY
           && currentY <= gateMaxY) {

          return true;
        }
      }
    }
  }

  /*
   * どのゲートにも
   * 遮られていない。
   */
  return false;
}

/**
 * @brief 状態を配列インデックスへ変換
 */
int DijkstraRoutePlanner::stateToIndex(
    int x,
    int y,
    Direction direction) const
{
  /*
   * 実座標
   *
   * 0,2,4,6,8,10
   *
   * ↓
   *
   * グリッド番号
   *
   * 0,1,2,3,4,5
   */

  const int gridX =
      x / MOVE_STEP;

  const int gridY =
      y / MOVE_STEP;

  /*
   * 例:
   *
   * gridX = 2
   * gridY = 3
   * direction = EAST
   *
   * ↓
   *
   *  ((3 * 6 + 2) * 4) + 1
   */

  return
      ((gridY * GRID_SIZE + gridX)
       * DIRECTION_COUNT)
      + static_cast<int>(
          direction);
}

/**
 * @brief 配列インデックスを状態へ変換
 */
RouteState DijkstraRoutePlanner::indexToState(
    int index) const
{
  /*
   * 方向を取り出す
   */
  const int directionValue =
      index % DIRECTION_COUNT;

  /*
   * 方向部分を取り除く
   */
  index /= DIRECTION_COUNT;

  /*
   * X方向のグリッド番号
   */
  const int gridX =
      index % GRID_SIZE;

  /*
   * Y方向のグリッド番号
   */
  const int gridY =
      index / GRID_SIZE;

  /*
   * グリッド番号を
   * 実座標へ戻す。
   */
  return {
      gridX * MOVE_STEP,
      gridY * MOVE_STEP,
      static_cast<Direction>(
          directionValue)
  };
}
