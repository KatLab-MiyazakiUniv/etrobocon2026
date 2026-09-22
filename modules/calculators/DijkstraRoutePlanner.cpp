/**
 * @file   DijkstraRoutePlanner.cpp
 * @brief  ゲートを考慮したダイクストラ法による経路探索
 * @author okuyama0528 yutaro-1214
 */

#include "DijkstraRoutePlanner.h"

namespace {

  /**
   * @brief ラジアンから度へ変換する係数
   */
  constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

  /**
   * @brief ゲート攻略に必要なゲート数
   */
  constexpr int REQUIRED_GATE_COUNT = 3;

  /**
   * @brief ETラリー開始X座標
   */
  constexpr int DEFAULT_START_X = 0;

  /**
   * @brief ETラリー開始Y座標
   */
  constexpr int DEFAULT_START_Y = 4;

  /**
   * @brief ETラリー開始方向
   */
  constexpr Direction DEFAULT_START_DIRECTION = Direction::LEFT;

  /**
   * @brief ゲート情報が不足している場合のゴールX座標
   */
  constexpr int DEFAULT_GOAL_X = 8;

  /**
   * @brief ゲート情報が不足している場合のゴールY座標
   */
  constexpr int DEFAULT_GOAL_Y = 0;

  /**
   * @brief ゲート情報が不足している場合のゴール方向
   */
  constexpr Direction DEFAULT_GOAL_DIRECTION = Direction::LEFT;

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
   *
   * ゲート足付近での通常の回頭を
   * 少し避けるためのペナルティ
   */
  constexpr int GATE_NEAR_TURN_PENALTY = 1;

  /**
   * @brief ゲート直前でゲート方向へ回頭する場合の追加コスト
   *
   * QR①付近まで別方向から進み、
   * ゲート直前で回頭する経路を強く避ける。
   * 通行禁止にはしないため、
   * 他に経路が存在しない場合は
   * この経路を選択することができる。
   */
  constexpr int GATE_APPROACH_TURN_PENALTY = 10;

  /**
   * @brief ゲート直前と判定する距離[マス]
   *
   * ゲートまで1マス以内でゲート方向へ回頭すると、
   * QR①の助走距離が足りなくなる可能性があるため
   * ペナルティを与える。
   */
  constexpr int GATE_APPROACH_DISTANCE = 1;

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

  /*
   * ETラリー開始位置は
   * デフォルトで(0, 4)、LEFTとする。
   */
  const int searchStartX = DEFAULT_START_X;
  const int searchStartY = DEFAULT_START_Y;
  const Direction searchStartDirection = DEFAULT_START_DIRECTION;

  /*
   * ゲート情報が3つ未満の場合は、
   * ゲート攻略を行わずデフォルトゴールへ向かう。
   */
  Point searchGoal = goal;
  Direction searchGoalDirection = goalDirection;

  if(gates.size() < REQUIRED_GATE_COUNT) {
    searchGoal = { DEFAULT_GOAL_X, DEFAULT_GOAL_Y };
    searchGoalDirection = DEFAULT_GOAL_DIRECTION;
  }

  // 使用しない開始位置の引数
  (void)startX;
  (void)startY;
  (void)startDirection;

  // 開始位置確認
  if(!isValid(searchStartX, searchStartY)) {
    return result;
  }

  // ゴール位置確認
  if(!isValid(searchGoal.x, searchGoal.y)) {
    return result;
  }

  /*
   * 探索する状態の総数
   *
   * X座標、Y座標、向いている方向の組み合わせで
   * 1つの状態を表す。
   */
  constexpr int STATE_COUNT = GRID_SIZE * GRID_SIZE * DIRECTION_COUNT;

  std::vector<int> distance(STATE_COUNT, INT_MAX);  // スタート地点から各状態までの最小コスト
  std::vector<int> previous(STATE_COUNT, -1);       // 各状態に来る直前の状態

  // 探索する状態をコストの小さい順に取り出すための優先度付きキュー
  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> queue;

  // 開始位置と開始方向を配列で使用するインデックスに変換
  const int startIndex = stateToIndex(searchStartX, searchStartY, searchStartDirection);

  distance[startIndex] = 0;
  queue.push({ 0, startIndex });

  /**
   * @brief 各方向へ移動するときのX座標変化量
   */
  constexpr int DX[DIRECTION_COUNT] = { 0, -MOVE_STEP, 0, MOVE_STEP };

  /**
   * @brief 各方向へ移動するときのY座標変化量
   */
  constexpr int DY[DIRECTION_COUNT] = { -MOVE_STEP, 0, MOVE_STEP, 0 };

  // ダイクストラ法
  while(!queue.empty()) {
    const QueueNode currentNode = queue.top();
    queue.pop();

    // すでにより低コストな経路が見つかっている場合は処理しない
    if(currentNode.cost != distance[currentNode.index]) {
      continue;
    }

    const RouteState current = indexToState(currentNode.index);

    // 現在位置から4方向への移動を確認
    for(int directionValue = 0; directionValue < DIRECTION_COUNT; ++directionValue) {
      const Direction nextDirection = static_cast<Direction>(directionValue);
      const int nextX = current.x + DX[directionValue];
      const int nextY = current.y + DY[directionValue];

      // マップ範囲外への移動は禁止
      if(!isValid(nextX, nextY)) {
        continue;
      }

      // ゲートを横切る移動は禁止
      if(isBlockedMove(current.x, current.y, nextX, nextY)) {
        continue;
      }

      // 移動コストを計算
      const int moveCost
          = calculateMoveCost(current.x, current.y, current.direction, nextDirection);
      const int nextCost = currentNode.cost + moveCost;
      const int nextIndex = stateToIndex(nextX, nextY, nextDirection);

      // より低コストな経路の場合は更新
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

    const int index = stateToIndex(searchGoal.x, searchGoal.y, arrivalDirection);

    // ゴールへ到達できない方向は使用しない
    if(distance[index] == INT_MAX) {
      continue;
    }

    // ゴール到着後に必要な最終回頭コスト
    int finalTurnCost = calculateTurnCost(arrivalDirection, searchGoalDirection);

    // ゴール位置で最終回頭するときのゲート足との接触判定
    if(arrivalDirection != searchGoalDirection && !isOuterArea(searchGoal.x, searchGoal.y)
       && isTurnBlockedByGatePost(searchGoal.x, searchGoal.y, arrivalDirection,
                                  searchGoalDirection)) {
      finalTurnCost = NEAR_GATE_TURN_COST;
    }

    // ゴール位置がゲート足付近の場合は回頭コストを追加
    if(arrivalDirection != searchGoalDirection && !isOuterArea(searchGoal.x, searchGoal.y)
       && isNearGatePost(searchGoal.x, searchGoal.y)) {
      finalTurnCost += GATE_NEAR_TURN_PENALTY;
    }

    // QR①付近でゲート方向へ回頭する場合は追加コストを与える
    if(arrivalDirection != searchGoalDirection
       && isTurningTowardGateNearEntrance(gates, searchGoal.x, searchGoal.y, searchGoalDirection)) {
      finalTurnCost += GATE_APPROACH_TURN_PENALTY;
    }

    const int finalCost = distance[index] + finalTurnCost;

    // より低コストな到着方法を保存
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

  // ゴールからスタートの順で格納されているため反転
  std::reverse(route.begin(), route.end());

  // ゴール地点で最終回頭
  if(route.back().direction != searchGoalDirection) {
    route.push_back({ searchGoal.x, searchGoal.y, searchGoalDirection });
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

  // 回頭前・回頭後の後方向
  const double currentRearX = -static_cast<double>(currentForward.x);
  const double currentRearY = -static_cast<double>(currentForward.y);
  const double nextRearX = -static_cast<double>(nextForward.x);
  const double nextRearY = -static_cast<double>(nextForward.y);

  // 回頭前・回頭後の後方向の角度
  const double currentRearAngle = std::atan2(currentRearY, currentRearX) * RAD_TO_DEG;
  const double nextRearAngle = std::atan2(nextRearY, nextRearX) * RAD_TO_DEG;

  // 回頭方向と回頭角度
  const double totalTurn = AngleNormalizer::normalizeAngle(nextRearAngle - currentRearAngle);

  // 各ゲート足を確認
  for(const Gate& gate : gates) {
    const Point posts[] = { gate.start, gate.end };

    for(const Point& post : posts) {
      const double dx = static_cast<double>(post.x - x);
      const double dy = static_cast<double>(post.y - y);
      const double postDistance = std::hypot(dx, dy);

      // 後部の旋回範囲より外側の場合は接触しない
      if(postDistance > TURN_SWEEP_RADIUS + TURN_SWEEP_MARGIN) {
        continue;
      }

      // ゲート足のすぐ近くは接触すると判定
      if(postDistance <= TURN_SWEEP_MARGIN) {
        return true;
      }

      // ゲート足の角度
      const double postAngle = std::atan2(dy, dx) * RAD_TO_DEG;

      // 回頭開始位置から見たゲート足の角度
      const double postTurn = AngleNormalizer::normalizeAngle(postAngle - currentRearAngle);

      // 180度回頭の場合は旋回範囲内にあるゲート足と接触すると判定
      if(std::abs(std::abs(totalTurn) - 180.0) < 0.01) {
        return true;
      }

      // 90度回頭の場合の旋回範囲判定
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
  // 各ゲート足を確認
  for(const Gate& gate : gates) {
    const Point posts[] = { gate.start, gate.end };

    for(const Point& post : posts) {
      const int dx = std::abs(x - post.x);
      const int dy = std::abs(y - post.y);

      // ゲート足からMOVE_STEP以内をゲート付近と判定
      if(dx <= MOVE_STEP && dy <= MOVE_STEP) {
        return true;
      }
    }
  }

  return false;
}

bool DijkstraRoutePlanner::isOuterArea(int x, int y) const
{
  // マップ外周上に存在するか判定
  return x == MAP_MIN || x == MAP_MAX || y == MAP_MIN || y == MAP_MAX;
}

bool DijkstraRoutePlanner::isValid(int x, int y) const
{
  // マップ範囲外の場合
  if(x < MAP_MIN || x > MAP_MAX || y < MAP_MIN || y > MAP_MAX) {
    return false;
  }

  // MOVE_STEP単位の座標でない場合
  if(x % MOVE_STEP != 0 || y % MOVE_STEP != 0) {
    return false;
  }

  return true;
}

bool DijkstraRoutePlanner::isBlockedMove(int currentX, int currentY, int nextX, int nextY) const
{
  // 各ゲートとの交差を確認
  for(const Gate& gate : gates) {
    // 上下移動
    if(currentX == nextX) {
      const int middleY = (currentY + nextY) / 2;

      // 横向きゲート
      if(gate.start.y == gate.end.y) {
        const int minX = std::min(gate.start.x, gate.end.x);
        const int maxX = std::max(gate.start.x, gate.end.x);

        // 移動経路がゲートを横切る場合
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

        // 移動経路がゲートを横切る場合
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
  // 座標をグリッド番号へ変換
  const int gridX = x / MOVE_STEP;
  const int gridY = y / MOVE_STEP;

  // X座標、Y座標、方向を1つのインデックスへ変換
  return ((gridY * GRID_SIZE + gridX) * DIRECTION_COUNT) + static_cast<int>(direction);
}

RouteState DijkstraRoutePlanner::indexToState(int index) const
{
  // インデックスから方向を取得
  const int directionValue = index % DIRECTION_COUNT;

  index /= DIRECTION_COUNT;

  // インデックスからグリッド座標を取得
  const int gridX = index % GRID_SIZE;
  const int gridY = index / GRID_SIZE;

  // グリッド座標を実際の座標へ変換
  return { gridX * MOVE_STEP, gridY * MOVE_STEP, static_cast<Direction>(directionValue) };
}