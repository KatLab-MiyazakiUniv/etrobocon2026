/**
 * @file   DijkstraRoutePlanner.cpp
 * @brief  ゲートを考慮したダイクストラ法による経路探索
 * @author okuyama0528 yutaro-1214
 */

#include "DijkstraRoutePlanner.h"

/**
 * @brief ダイクストラ法の優先度付きキューで使用するノード
 */
namespace {

  struct QueueNode {
    int cost;
    int index;

    bool operator>(
        const QueueNode& other) const
    {
      return cost > other.cost;
    }
  };

  constexpr double PI
      = 3.14159265358979323846;

}  // namespace

DijkstraRoutePlanner::DijkstraRoutePlanner(
    const std::vector<Gate>& gates)
  : gates(gates)
{
}

RouteResult DijkstraRoutePlanner::search(
    int startX,
    int startY,
    Direction startDirection,
    const Point& goal,
    Direction goalDirection)
{
  RouteResult result;

  // =====================================================
  // 開始位置確認
  // =====================================================

  if(!isValid(
         startX,
         startY)) {

    return result;
  }

  // =====================================================
  // ゴール位置確認
  // =====================================================

  if(!isValid(
         goal.x,
         goal.y)) {

    return result;
  }

  constexpr int STATE_COUNT
      = GRID_SIZE
        * GRID_SIZE
        * DIRECTION_COUNT;

  std::vector<int> distance(
      STATE_COUNT,
      INT_MAX);

  std::vector<int> previous(
      STATE_COUNT,
      -1);

  std::priority_queue<
      QueueNode,
      std::vector<QueueNode>,
      std::greater<QueueNode>>
      queue;

  const int startIndex
      = stateToIndex(
          startX,
          startY,
          startDirection);

  distance[startIndex] = 0;

  queue.push({
      0,
      startIndex
  });

  /*
   * 座標系
   *
   * UP    : Y - 2
   * RIGHT : X - 2
   * DOWN  : Y + 2
   * LEFT  : X + 2
   */
  constexpr int DX[DIRECTION_COUNT] = {
      0,
      -MOVE_STEP,
      0,
      MOVE_STEP
  };

  constexpr int DY[DIRECTION_COUNT] = {
      -MOVE_STEP,
      0,
      MOVE_STEP,
      0
  };

  // =====================================================
  // ダイクストラ法
  // =====================================================

  while(!queue.empty()) {

    const QueueNode currentNode
        = queue.top();

    queue.pop();

    if(currentNode.cost
       != distance[currentNode.index]) {

      continue;
    }

    const RouteState current
        = indexToState(
            currentNode.index);

    // ===================================================
    // 4方向を探索
    // ===================================================

    for(int directionValue = 0;
        directionValue < DIRECTION_COUNT;
        ++directionValue) {

      const Direction nextDirection
          = static_cast<Direction>(
              directionValue);

      const int nextX
          = current.x
            + DX[directionValue];

      const int nextY
          = current.y
            + DY[directionValue];

      // -------------------------------------------------
      // 範囲外
      // -------------------------------------------------

      if(!isValid(
             nextX,
             nextY)) {

        continue;
      }

      // -------------------------------------------------
      // ゲートを横切る移動は禁止
      // -------------------------------------------------

      if(isBlockedMove(
             current.x,
             current.y,
             nextX,
             nextY)) {

        continue;
      }

      // -------------------------------------------------
      // 移動コスト
      // -------------------------------------------------

      const int moveCost
          = calculateMoveCost(
              current.x,
              current.y,
              current.direction,
              nextDirection);

      const int nextCost
          = currentNode.cost
            + moveCost;

      const int nextIndex
          = stateToIndex(
              nextX,
              nextY,
              nextDirection);

      // -------------------------------------------------
      // より低コストなら更新
      // -------------------------------------------------

      if(nextCost
         < distance[nextIndex]) {

        distance[nextIndex]
            = nextCost;

        previous[nextIndex]
            = currentNode.index;

        queue.push({
            nextCost,
            nextIndex
        });
      }
    }
  }

  // =====================================================
  // ゴールへの4方向の到着方法を比較
  // =====================================================

  int bestIndex = -1;

  int bestCost = INT_MAX;

  for(int directionValue = 0;
      directionValue < DIRECTION_COUNT;
      ++directionValue) {

    const Direction arrivalDirection
        = static_cast<Direction>(
            directionValue);

    const int index
        = stateToIndex(
            goal.x,
            goal.y,
            arrivalDirection);

    if(distance[index]
       == INT_MAX) {

      continue;
    }

    int finalTurnCost
        = calculateTurnCost(
            arrivalDirection,
            goalDirection);

    // ===================================================
    // ゴール位置で最終回頭するときの接触判定
    // ===================================================

    if(arrivalDirection
           != goalDirection
       && !isOuterArea(
              goal.x,
              goal.y)
       && isTurnBlockedByGatePost(
              goal.x,
              goal.y,
              arrivalDirection,
              goalDirection)) {

      finalTurnCost
          = NEAR_GATE_TURN_COST;
    }

    const int finalCost
        = distance[index]
          + finalTurnCost;

    if(finalCost
       < bestCost) {

      bestCost
          = finalCost;

      bestIndex
          = index;
    }
  }

  // =====================================================
  // 経路なし
  // =====================================================

  if(bestIndex == -1) {
    return result;
  }

  // =====================================================
  // 経路復元
  // =====================================================

  std::vector<RouteState> route;

  for(int index = bestIndex;
      index != -1;
      index = previous[index]) {

    route.push_back(
        indexToState(
            index));
  }

  std::reverse(
      route.begin(),
      route.end());

  // =====================================================
  // ゴール地点で最終回頭
  // =====================================================

  if(route.back().direction
     != goalDirection) {

    route.push_back({
        goal.x,
        goal.y,
        goalDirection
    });
  }

  result.found = true;
  result.cost = bestCost;
  result.route = route;

  return result;
}

int DijkstraRoutePlanner::calculateTurnCost(
    Direction currentDirection,
    Direction nextDirection) const
{
  const int current
      = static_cast<int>(
          currentDirection);

  const int next
      = static_cast<int>(
          nextDirection);

  int difference
      = std::abs(
          current - next);

  difference
      = std::min(
          difference,
          DIRECTION_COUNT
              - difference);

  // =====================================================
  // 同じ方向
  // =====================================================

  if(difference == 0) {
    return 0;
  }

  // =====================================================
  // 90度回頭
  // =====================================================

  if(difference == 1) {
    return TURN_90_COST;
  }

  // =====================================================
  // 180度回頭
  // =====================================================

  return TURN_180_COST;
}

int DijkstraRoutePlanner::calculateMoveCost(
    int currentX,
    int currentY,
    Direction currentDirection,
    Direction nextDirection) const
{
  int turnCost
      = calculateTurnCost(
          currentDirection,
          nextDirection);

  // =====================================================
  // 回頭なし
  // =====================================================

  if(turnCost == 0) {

    return STRAIGHT_COST;
  }

  // =====================================================
  // 回頭時にロボット後部がゲート足へ
  // 接触する可能性がある場合
  // =====================================================

  if(!isOuterArea(
         currentX,
         currentY)
     && isTurnBlockedByGatePost(
         currentX,
         currentY,
         currentDirection,
         nextDirection)) {

    turnCost
        = NEAR_GATE_TURN_COST;
  }

  return turnCost
         + STRAIGHT_COST;
}

bool DijkstraRoutePlanner::isTurnBlockedByGatePost(
    int x,
    int y,
    Direction currentDirection,
    Direction nextDirection) const
{
  // =====================================================
  // 回頭しない場合
  // =====================================================

  if(currentDirection
     == nextDirection) {

    return false;
  }

  // =====================================================
  // 回頭前・回頭後の前方向
  // =====================================================

  const Point currentForward
      = directionToVector(
          currentDirection);

  const Point nextForward
      = directionToVector(
          nextDirection);

  // =====================================================
  // 後方向
  //
  // 回頭中心が前側なので、
  // 回頭時には後部がこの方向へ伸びる。
  // =====================================================

  const double currentRearX
      = -static_cast<double>(
          currentForward.x);

  const double currentRearY
      = -static_cast<double>(
          currentForward.y);

  const double nextRearX
      = -static_cast<double>(
          nextForward.x);

  const double nextRearY
      = -static_cast<double>(
          nextForward.y);

  // =====================================================
  // 後方向の角度
  // =====================================================

  const double currentRearAngle
      = std::atan2(
          currentRearY,
          currentRearX);

  const double nextRearAngle
      = std::atan2(
          nextRearY,
          nextRearX);

  // =====================================================
  // 回頭方向
  // =====================================================

  const double totalTurn
      = normalizeRad(
          nextRearAngle
          - currentRearAngle);

  // =====================================================
  // 各ゲート足を確認
  // =====================================================

  for(const Gate& gate :
      gates) {

    const Point posts[] = {
        gate.start,
        gate.end
    };

    for(const Point& post :
        posts) {

      // -------------------------------------------------
      // 回頭中心からゲート足まで
      // -------------------------------------------------

      const double dx
          = static_cast<double>(
              post.x - x);

      const double dy
          = static_cast<double>(
              post.y - y);

      const double postDistance
          = std::hypot(
              dx,
              dy);

      // -------------------------------------------------
      // 後部の旋回範囲より外なら安全
      // -------------------------------------------------

      if(postDistance
         > TURN_SWEEP_RADIUS
               + TURN_SWEEP_MARGIN) {

        continue;
      }

      // -------------------------------------------------
      // 回頭中心のすぐ近くは危険
      // -------------------------------------------------

      if(postDistance
         <= TURN_SWEEP_MARGIN) {

        return true;
      }

      // -------------------------------------------------
      // ゲート足の角度
      // -------------------------------------------------

      const double postAngle
          = std::atan2(
              dy,
              dx);

      const double postTurn
          = normalizeRad(
              postAngle
              - currentRearAngle);

      // =================================================
      // 180度回頭
      // =================================================

      if(std::abs(
             std::abs(totalTurn)
             - PI)
         < 0.01) {

        /*
         * 180度回頭の場合は、
         * 後部が半円を通る。
         *
         * どちら回りになるかが
         * この経路探索だけでは決められないため、
         * 半径内に足があれば危険として扱う。
         */
        return true;
      }

      // =================================================
      // 90度回頭
      // =================================================

      bool insideSweep = false;

      /*
       * currentRearAngleからnextRearAngleへ
       * 実際に回頭する角度範囲内に
       * ゲート足が存在するか確認する。
       */
      if(totalTurn > 0.0) {

        insideSweep
            = postTurn >= 0.0
              && postTurn <= totalTurn;

      } else {

        insideSweep
            = postTurn <= 0.0
              && postTurn >= totalTurn;
      }

      if(insideSweep) {

        return true;
      }
    }
  }

  return false;
}

Point DijkstraRoutePlanner::directionToVector(
    Direction direction) const
{
  /*
   * このプロジェクトの座標系
   *
   * UP    : Y -
   * RIGHT : X -
   * DOWN  : Y +
   * LEFT  : X +
   */

  switch(direction) {

    case Direction::UP:
      return { 0, -1 };

    case Direction::RIGHT:
      return { -1, 0 };

    case Direction::DOWN:
      return { 0, 1 };

    case Direction::LEFT:
      return { 1, 0 };
  }

  return { 0, 0 };
}

double DijkstraRoutePlanner::normalizeRad(
    double angle) const
{
  while(angle > PI) {

    angle
        -= 2.0 * PI;
  }

  while(angle < -PI) {

    angle
        += 2.0 * PI;
  }

  return angle;
}

bool DijkstraRoutePlanner::isNearGatePost(
    int x,
    int y) const
{
  for(const Gate& gate :
      gates) {

    const Point posts[] = {
        gate.start,
        gate.end
    };

    for(const Point& post :
        posts) {

      const int dx
          = std::abs(
              x - post.x);

      const int dy
          = std::abs(
              y - post.y);

      if(dx <= MOVE_STEP
         && dy <= MOVE_STEP) {

        return true;
      }
    }
  }

  return false;
}

bool DijkstraRoutePlanner::isOuterArea(
    int x,
    int y) const
{
  return x == MAP_MIN
         || x == MAP_MAX
         || y == MAP_MIN
         || y == MAP_MAX;
}

bool DijkstraRoutePlanner::isValid(
    int x,
    int y) const
{
  if(x < MAP_MIN
     || x > MAP_MAX
     || y < MAP_MIN
     || y > MAP_MAX) {

    return false;
  }

  if(x % MOVE_STEP != 0
     || y % MOVE_STEP != 0) {

    return false;
  }

  return true;
}

bool DijkstraRoutePlanner::isBlockedMove(
    int currentX,
    int currentY,
    int nextX,
    int nextY) const
{
  for(const Gate& gate :
      gates) {

    // =====================================================
    // 上下移動
    // =====================================================

    if(currentX
       == nextX) {

      const int middleY
          = (currentY
             + nextY)
            / 2;

      // -------------------------------------------------
      // 横向きゲート
      // -------------------------------------------------

      if(gate.start.y
         == gate.end.y) {

        const int minX
            = std::min(
                gate.start.x,
                gate.end.x);

        const int maxX
            = std::max(
                gate.start.x,
                gate.end.x);

        if(middleY
               == gate.start.y
           && currentX >= minX
           && currentX <= maxX) {

          return true;
        }
      }
    }

    // =====================================================
    // 左右移動
    // =====================================================

    if(currentY
       == nextY) {

      const int middleX
          = (currentX
             + nextX)
            / 2;

      // -------------------------------------------------
      // 縦向きゲート
      // -------------------------------------------------

      if(gate.start.x
         == gate.end.x) {

        const int minY
            = std::min(
                gate.start.y,
                gate.end.y);

        const int maxY
            = std::max(
                gate.start.y,
                gate.end.y);

        if(middleX
               == gate.start.x
           && currentY >= minY
           && currentY <= maxY) {

          return true;
        }
      }
    }
  }

  return false;
}

int DijkstraRoutePlanner::stateToIndex(
    int x,
    int y,
    Direction direction) const
{
  const int gridX
      = x / MOVE_STEP;

  const int gridY
      = y / MOVE_STEP;

  return (
      (gridY
       * GRID_SIZE
       + gridX)
      * DIRECTION_COUNT)
      + static_cast<int>(
          direction);
}

RouteState DijkstraRoutePlanner::indexToState(
    int index) const
{
  const int directionValue
      = index
        % DIRECTION_COUNT;

  index
      /= DIRECTION_COUNT;

  const int gridX
      = index
        % GRID_SIZE;

  const int gridY
      = index
        / GRID_SIZE;

  return {
      gridX * MOVE_STEP,
      gridY * MOVE_STEP,
      static_cast<Direction>(
          directionValue)
  };
}