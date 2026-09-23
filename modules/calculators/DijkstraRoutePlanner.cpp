/**
 * @file   DijkstraRoutePlanner.cpp
 * @brief  ゲートを考慮したダイクストラ法による経路探索
 * @author okuyama0528 yutaro-1214
 */

#include "DijkstraRoutePlanner.h"

#include "SystemInfo.h"

namespace {

  /**
   * @brief ダイクストラ法の優先度付きキューで使用するノード
   */
  struct QueueNode {
    int cost;
    int index;

    bool operator>(const QueueNode& other) const { return cost > other.cost; }
  };

  /**
   * @brief ゲート付近で回頭した場合の追加コスト
   * ゲート足付近での通常の回頭を
   * 少し避けるためのペナルティ
   */
  constexpr int GATE_NEAR_TURN_PENALTY = 1;

  /**
   * @brief ゲート直前でゲート方向へ回頭する場合の追加コスト
   * QR①付近まで別方向から進み、
   * ゲート直前で回頭する経路を強く避ける。
   * 通行禁止にはしないため、
   * 他に経路が存在しない場合は
   * この経路を選択することができる。
   */
  constexpr int GATE_APPROACH_TURN_PENALTY = 10;

  /**
   * @brief ゲート直前と判定する距離[マス]
   * ゲートまで1マス以内でゲート方向へ回頭すると、
   * QR①の助走距離が足りなくなる可能性があるため
   * ペナルティを与える。
   */
  constexpr int GATE_APPROACH_DISTANCE = 1;

  /**
   * @brief LEFTを向く回頭を禁止する地点か判定する
   * @param x X座標
   * @param y Y座標
   * @return true LEFTを向く回頭を禁止する地点
   * @return false LEFTを向く回頭が可能な地点
   */
  bool isTurnForbiddenPoint(int x, int y)
  {
    if(x != 0) {
      return false;
    }

    return y == 2 || y == 6 || y == 10;
  }

  /**
   * @brief 外周ゲートの外側にある入口か判定する
   * @param gates ゲート一覧
   * @param goal 入口候補座標
   * @param goalDirection ゲート通過方向
   * @return true 外周ゲートの外側にある入口
   * @return false それ以外
   *
   * 外周ゲートではマップ内側からのみ進入するため、
   * 外側から内側へ通過する入口候補を無効にする。
   */
  bool isOuterGateOutsideEntrance(const std::vector<Gate>& gates, const Point& goal,
                                  Direction goalDirection)
  {
    for(const Gate& gate : gates) {
      // 縦向きゲート
      if(gate.start.x == gate.end.x) {
        const int gateX = gate.start.x;
        const int centerY = (gate.start.y + gate.end.y) / 2;

        // 左外周ゲート
        if(gateX == 1) {
          /*
           * 外側入口
           *
           * (0, centerY)
           *      ↓
           * LEFT方向へ進むとマップ内側へ入る。
           *
           * LEFT : X+
           */
          if(goal.x == 0 && goal.y == centerY && goalDirection == Direction::LEFT) {
            return true;
          }
        }

        // 右外周ゲート
        if(gateX == SystemInfo::X_GRID_NUM - 1) {
          /*
           * 外側入口
           *
           * (X_GRID_NUM, centerY)
           *      ↓
           * RIGHT方向へ進むとマップ内側へ入る。
           *
           * RIGHT : X-
           */
          if(goal.x == SystemInfo::X_GRID_NUM && goal.y == centerY
             && goalDirection == Direction::RIGHT) {
            return true;
          }
        }
      }

      // 横向きゲート
      if(gate.start.y == gate.end.y) {
        const int centerX = (gate.start.x + gate.end.x) / 2;
        const int gateY = gate.start.y;

        // 上外周ゲート
        if(gateY == 1) {
          /*
           * 外側入口
           *
           * (centerX, 0)
           *      ↓
           * DOWN方向へ進むとマップ内側へ入る。
           */
          if(goal.x == centerX && goal.y == 0 && goalDirection == Direction::DOWN) {
            return true;
          }
        }

        // 下外周ゲート
        if(gateY == SystemInfo::Y_GRID_NUM - 1) {
          /*
           * 外側入口
           *
           * (centerX, Y_GRID_NUM)
           *      ↓
           * UP方向へ進むとマップ内側へ入る。
           */
          if(goal.x == centerX && goal.y == SystemInfo::Y_GRID_NUM
             && goalDirection == Direction::UP) {
            return true;
          }
        }
      }
    }

    return false;
  }

  /**
   * @brief QR①付近からゲート方向を向くか判定する
   * @param gates ゲート一覧
   * @param currentX 現在X座標
   * @param currentY 現在Y座標
   * @param nextDirection 回頭後の方向
   * @return true QR①付近からゲート方向を向く
   * @return false それ以外
   */
  bool isTurningTowardGateNearEntrance(const std::vector<Gate>& gates, int currentX, int currentY,
                                       Direction nextDirection)
  {
    for(const Gate& gate : gates) {
      // 縦向きゲート
      if(gate.start.x == gate.end.x) {
        const int gateX = gate.start.x;
        const int minY = std::min(gate.start.y, gate.end.y);
        const int maxY = std::max(gate.start.y, gate.end.y);

        // ゲートの通過範囲と同じY位置にいるか
        if(currentY >= minY && currentY <= maxY) {
          const int distanceToGate = std::abs(currentX - gateX);

          // ゲートまで1マス以内ならQR①直前と判定
          if(distanceToGate <= GATE_APPROACH_DISTANCE) {
            // ゲートの右側からゲートへ向かう
            if(currentX > gateX && nextDirection == Direction::RIGHT) {
              return true;
            }

            // ゲートの左側からゲートへ向かう
            if(currentX < gateX && nextDirection == Direction::LEFT) {
              return true;
            }
          }
        }
      }

      // 横向きゲート
      if(gate.start.y == gate.end.y) {
        const int gateY = gate.start.y;

        const int minX = std::min(gate.start.x, gate.end.x);
        const int maxX = std::max(gate.start.x, gate.end.x);

        // ゲートの通過範囲と同じX位置にいるか
        if(currentX >= minX && currentX <= maxX) {
          const int distanceToGate = std::abs(currentY - gateY);

          // ゲートまで1マス以内ならQR①直前と判定
          if(distanceToGate <= GATE_APPROACH_DISTANCE) {
            // ゲートの下側からゲートへ向かう
            if(currentY > gateY && nextDirection == Direction::UP) {
              return true;
            }

            // ゲートの上側からゲートへ向かう
            if(currentY < gateY && nextDirection == Direction::DOWN) {
              return true;
            }
          }
        }
      }
    }

    return false;
  }

}  // namespace

DijkstraRoutePlanner::DijkstraRoutePlanner(const std::vector<Gate>& gates) : gates(gates) {}

RouteResult DijkstraRoutePlanner::search(int startX, int startY, Direction startDirection,
                                         const Point& goal, Direction goalDirection)
{
  RouteResult result;

  // 開始位置確認
  if(!isValid(startX, startY)) {
    return result;
  }

  // ゴール位置確認
  if(!isValid(goal.x, goal.y)) {
    return result;
  }

  /*
   * 外周ゲートではマップ内側からのみ進入する。
   *
   * 外側から内側へ通過するGatePassが
   * ゴールとして指定された場合は、
   * この入口候補を使用しない。
   */
  if(isOuterGateOutsideEntrance(gates, goal, goalDirection)) {
    return result;
  }

  // 探索する状態の総数
  // X座標、Y座標、向いている方向の組み合わせで状態を表す
  constexpr int STATE_COUNT = GRID_SIZE * GRID_SIZE * DIRECTION_COUNT;

  std::vector<int> distance(STATE_COUNT, INT_MAX);  // スタート地点から各状態までの最小コストを記録
  std::vector<int> previous(STATE_COUNT, -1);       // 各状態に来る直前の状態を記録

  // 探索する状態をコストの小さい順に取り出すための優先度付きキュー
  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> queue;

  // 開始位置と開始方向を、配列で使うインデックスに変換
  const int startIndex = stateToIndex(startX, startY, startDirection);

  distance[startIndex] = 0;
  queue.push({ 0, startIndex });

  constexpr int DX[DIRECTION_COUNT] = { 0, -MOVE_STEP, 0, MOVE_STEP };
  constexpr int DY[DIRECTION_COUNT] = { -MOVE_STEP, 0, MOVE_STEP, 0 };

  // ダイクストラ法
  while(!queue.empty()) {
    const QueueNode currentNode = queue.top();
    queue.pop();

    if(currentNode.cost != distance[currentNode.index]) {
      continue;
    }

    const RouteState current = indexToState(currentNode.index);

    for(int directionValue = 0; directionValue < DIRECTION_COUNT; ++directionValue) {
      const Direction nextDirection = static_cast<Direction>(directionValue);
      const int nextX = current.x + DX[directionValue];
      const int nextY = current.y + DY[directionValue];

      if(!isValid(nextX, nextY)) {
        continue;
      }

      /*
       * (0,2)、(0,6)、(0,10)では、
       * 回頭してLEFTを向く動作を禁止する。
       *
       * すでにLEFTを向いている状態での
       * LEFT方向への直進は許可する。
       */
      if(isTurnForbiddenPoint(current.x, current.y) && current.direction != nextDirection
         && nextDirection == Direction::LEFT) {
        continue;
      }

      // ゲートを横切る移動は禁止
      if(isBlockedMove(current.x, current.y, nextX, nextY)) {
        continue;
      }

      // 移動コスト
      const int moveCost
          = calculateMoveCost(current.x, current.y, current.direction, nextDirection);

      const int nextCost = currentNode.cost + moveCost;
      const int nextIndex = stateToIndex(nextX, nextY, nextDirection);

      // より低コストなら更新
      if(nextCost < distance[nextIndex]) {
        distance[nextIndex] = nextCost;
        previous[nextIndex] = currentNode.index;

        queue.push({ nextCost, nextIndex });
      }
    }
  }

  // ゴールへの4方向の到着方法を比較
  int bestIndex = -1;
  int bestCost = INT_MAX;

  for(int directionValue = 0; directionValue < DIRECTION_COUNT; ++directionValue) {
    const Direction arrivalDirection = static_cast<Direction>(directionValue);
    const int index = stateToIndex(goal.x, goal.y, arrivalDirection);

    if(distance[index] == INT_MAX) {
      continue;
    }

    /*
     * (0,2)、(0,6)、(0,10)では、
     * ゴール地点に到着してから
     * LEFTへ回頭する動作も禁止する。
     *
     * LEFTを向いた状態で到着している場合は
     * 回頭が発生しないため許可する。
     */
    if(isTurnForbiddenPoint(goal.x, goal.y) && arrivalDirection != goalDirection
       && goalDirection == Direction::LEFT) {
      continue;
    }

    int finalTurnCost = calculateTurnCost(arrivalDirection, goalDirection);

    // ゴール位置で最終回頭するときの接触判定
    if(arrivalDirection != goalDirection && !isOuterArea(goal.x, goal.y)
       && isTurnBlockedByGatePost(goal.x, goal.y, arrivalDirection, goalDirection)) {
      finalTurnCost = NEAR_GATE_TURN_COST;
    }

    // ゴール位置がゲート付近の場合
    if(arrivalDirection != goalDirection && !isOuterArea(goal.x, goal.y)
       && isNearGatePost(goal.x, goal.y)) {
      finalTurnCost += GATE_NEAR_TURN_PENALTY;
    }

    // QR①付近でゲート方向へ回頭する場合
    if(arrivalDirection != goalDirection
       && isTurningTowardGateNearEntrance(gates, goal.x, goal.y, goalDirection)) {
      finalTurnCost += GATE_APPROACH_TURN_PENALTY;
    }

    const int finalCost = distance[index] + finalTurnCost;

    if(finalCost < bestCost) {
      bestCost = finalCost;
      bestIndex = index;
    }
  }

  // 経路なし
  if(bestIndex == -1) {
    return result;
  }

  // 経路復元
  std::vector<RouteState> route;

  for(int index = bestIndex; index != -1; index = previous[index]) {
    route.push_back(indexToState(index));
  }

  std::reverse(route.begin(), route.end());

  // ゴール地点で最終回頭
  if(route.back().direction != goalDirection) {
    route.push_back({ goal.x, goal.y, goalDirection });
  }

  result.found = true;
  result.cost = bestCost;
  result.route = route;

  return result;
}

int DijkstraRoutePlanner::calculateTurnCost(Direction currentDirection,
                                            Direction nextDirection) const
{
  const int current = static_cast<int>(currentDirection);
  const int next = static_cast<int>(nextDirection);

  int difference = std::abs(current - next);
  difference = std::min(difference, DIRECTION_COUNT - difference);

  // 同じ方向
  if(difference == 0) {
    return 0;
  }

  // 90度回頭
  if(difference == 1) {
    return TURN_90_COST;
  }

  // 180度回頭
  return TURN_180_COST;
}

int DijkstraRoutePlanner::calculateMoveCost(int currentX, int currentY, Direction currentDirection,
                                            Direction nextDirection) const
{
  int turnCost = calculateTurnCost(currentDirection, nextDirection);

  // 回頭なし
  if(turnCost == 0) {
    return STRAIGHT_COST;
  }

  // 回頭時にロボット後部がゲート足へ接触する可能性がある場合
  if(!isOuterArea(currentX, currentY)
     && isTurnBlockedByGatePost(currentX, currentY, currentDirection, nextDirection)) {
    turnCost = NEAR_GATE_TURN_COST;
  }

  // ゲート足付近での回頭ペナルティ
  if(!isOuterArea(currentX, currentY) && isNearGatePost(currentX, currentY)) {
    turnCost += GATE_NEAR_TURN_PENALTY;
  }

  // QR①付近でゲート方向へ回頭する経路を避ける
  if(isTurningTowardGateNearEntrance(gates, currentX, currentY, nextDirection)) {
    turnCost += GATE_APPROACH_TURN_PENALTY;
  }

  return turnCost + STRAIGHT_COST;
}

bool DijkstraRoutePlanner::isTurnBlockedByGatePost(int x, int y, Direction currentDirection,
                                                   Direction nextDirection) const
{
  // 回頭しない場合
  if(currentDirection == nextDirection) {
    return false;
  }

  // 回頭前・回頭後の前方向
  const Point currentForward = directionToVector(currentDirection);
  const Point nextForward = directionToVector(nextDirection);

  // 後方向
  const double currentRearX = -static_cast<double>(currentForward.x);
  const double currentRearY = -static_cast<double>(currentForward.y);
  const double nextRearX = -static_cast<double>(nextForward.x);
  const double nextRearY = -static_cast<double>(nextForward.y);

  // 後方向の角度
  const double currentRearAngle = std::atan2(currentRearY, currentRearX) * RAD_TO_DEG;
  const double nextRearAngle = std::atan2(nextRearY, nextRearX) * RAD_TO_DEG;

  // 回頭方向
  const double totalTurn = AngleNormalizer::normalizeAngle(nextRearAngle - currentRearAngle);

  // 各ゲート足を確認
  for(const Gate& gate : gates) {
    const Point posts[] = { gate.start, gate.end };

    for(const Point& post : posts) {
      const double dx = static_cast<double>(post.x - x);
      const double dy = static_cast<double>(post.y - y);
      const double postDistance = std::hypot(dx, dy);

      // 後部の旋回範囲より外なら安全
      if(postDistance > TURN_SWEEP_RADIUS + TURN_SWEEP_MARGIN) {
        continue;
      }

      // ゲート足のすぐ近くは危険
      if(postDistance <= TURN_SWEEP_MARGIN) {
        return true;
      }

      // ゲート足の角度
      const double postAngle = std::atan2(dy, dx) * RAD_TO_DEG;
      const double postTurn = AngleNormalizer::normalizeAngle(postAngle - currentRearAngle);

      // 180度回頭
      if(std::abs(std::abs(totalTurn) - 180.0) < 0.01) {
        return true;
      }

      // 90度回頭
      bool insideSweep = false;

      if(totalTurn > 0.0) {
        insideSweep = postTurn >= 0.0 && postTurn <= totalTurn;

      } else {
        insideSweep = postTurn <= 0.0 && postTurn >= totalTurn;
      }

      if(insideSweep) {
        return true;
      }
    }
  }

  return false;
}

Point DijkstraRoutePlanner::directionToVector(Direction direction) const
{
  /*
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

bool DijkstraRoutePlanner::isNearGatePost(int x, int y) const
{
  for(const Gate& gate : gates) {
    const Point posts[] = { gate.start, gate.end };

    for(const Point& post : posts) {
      const int dx = std::abs(x - post.x);
      const int dy = std::abs(y - post.y);

      /*
       * ゲート足からMOVE_STEP以内を
       * ゲート付近と判定する
       */
      if(dx <= MOVE_STEP && dy <= MOVE_STEP) {
        return true;
      }
    }
  }

  return false;
}

bool DijkstraRoutePlanner::isOuterArea(int x, int y) const
{
  return x == MAP_MIN || x == MAP_MAX || y == MAP_MIN || y == MAP_MAX;
}

bool DijkstraRoutePlanner::isValid(int x, int y) const
{
  if(x < MAP_MIN || x > MAP_MAX || y < MAP_MIN || y > MAP_MAX) {
    return false;
  }

  if(x % MOVE_STEP != 0 || y % MOVE_STEP != 0) {
    return false;
  }

  return true;
}

bool DijkstraRoutePlanner::isBlockedMove(int currentX, int currentY, int nextX, int nextY) const
{
  for(const Gate& gate : gates) {
    // 上下移動
    if(currentX == nextX) {
      const int middleY = (currentY + nextY) / 2;

      // 横向きゲート
      if(gate.start.y == gate.end.y) {
        const int minX = std::min(gate.start.x, gate.end.x);
        const int maxX = std::max(gate.start.x, gate.end.x);

        if(middleY == gate.start.y && currentX >= minX && currentX <= maxX) {
          return true;
        }
      }
    }

    // 左右移動
    if(currentY == nextY) {
      const int middleX = (currentX + nextX) / 2;

      // 縦向きゲート
      if(gate.start.x == gate.end.x) {
        const int minY = std::min(gate.start.y, gate.end.y);
        const int maxY = std::max(gate.start.y, gate.end.y);

        if(middleX == gate.start.x && currentY >= minY && currentY <= maxY) {
          return true;
        }
      }
    }
  }

  return false;
}

int DijkstraRoutePlanner::stateToIndex(int x, int y, Direction direction) const
{
  const int gridX = x / MOVE_STEP;
  const int gridY = y / MOVE_STEP;

  return ((gridY * GRID_SIZE + gridX) * DIRECTION_COUNT) + static_cast<int>(direction);
}

RouteState DijkstraRoutePlanner::indexToState(int index) const
{
  const int directionValue = index % DIRECTION_COUNT;

  index /= DIRECTION_COUNT;

  const int gridX = index % GRID_SIZE;
  const int gridY = index / GRID_SIZE;

  return { gridX * MOVE_STEP, gridY * MOVE_STEP, static_cast<Direction>(directionValue) };
}