/**
 * @file   RouteFollower.cpp
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#include "RouteFollower.h"

namespace {

  /**
   * @brief 回頭終了判定許容誤差[deg]
   */
  constexpr double ROTATION_TOLERANCE = 2.0;

  /**
   * @brief QR①とQR②の距離[mm]
   */
  constexpr double QR_DISTANCE = 250.0;

  /**
   * @brief QRとゲート中央の距離[mm]
   */
  constexpr double QR_TO_GATE_DISTANCE = QR_DISTANCE / 2.0;

  /**
   * @brief QRを検出する基準距離[mm]
   */
  constexpr double SQUARE_DETECTION_DISTANCE = 250.0;

  /**
   * @brief QR①検出位置の許容誤差[mm]
   */
  constexpr double FIRST_DETECTION_TOLERANCE = 10.0;

  /**
   * @brief 外周ゲートでQR①位置から前進する距離[mm]
   */
  constexpr double OUTER_GATE_FORWARD_DISTANCE = 400.0;

  /**
   * @brief 動作切り替え待機時間[ms]
   */
  constexpr int MOTION_SWITCH_WAIT = 200;

}  // namespace

RouteFollower::RouteFollower(Robot& _robot, const EtRallyMap& _map, double _targetSpeed,
                             const Pid::PidGain& _rotationPid,
                             const Pid::PidGain& _squareRotationPid,
                             const Pid::PidGain& _straightAnglePid, double _straightDeadbandRate,
                             double _straightMaxoutRate)
  : robot(_robot),
    map(_map),
    targetSpeed(_targetSpeed),
    rotationPid(_rotationPid),
    squareRotationPid(_squareRotationPid),
    straightAnglePid(_straightAnglePid),
    straightDeadbandRate(_straightDeadbandRate),
    straightMaxoutRate(_straightMaxoutRate)
{
  LOG_CREATE("RouteFollower");
}

void RouteFollower::run(const std::vector<RouteState>& route)
{
  if(route.size() < 2) {
    Logger::warning("RouteFollower: "
                    "経路が2点未満のため走行できません");

    return;
  }

  for(std::size_t i = 1; i < route.size(); ++i) {
    // 区間の開始地点と終了地点を取得する。
    const RouteState& from = route[i - 1];
    const RouteState& to = route[i];

    Logger::printfLog(Logger::INFO,
                      "RouteFollower[%d]: "
                      "現在走行する区間 "
                      "(%d,%d) → (%d,%d)",
                      static_cast<int>(i), from.x, from.y, to.x, to.y);

    // 次の区間を走行するために必要な回頭角度を計算する。
    const double rotationAngle = calculateRotationAngle(from.direction, to.direction);

    // 区間開始時に回頭が必要だったかを判定する。
    const bool rotatedAtSegmentStart = std::abs(rotationAngle) > ROTATION_TOLERANCE;

    if(rotatedAtSegmentStart) {
      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "回頭開始 %.2f deg",
                        rotationAngle);

      rotate(rotationAngle);
    }

    // 同一座標の場合は回頭のみ行う。
    if(from.x == to.x && from.y == to.y) {
      continue;
    }

    // 区間距離を計算する。
    const double distance = calculateDistance(from, to);

    if(distance <= 0.0) {
      Logger::error("RouteFollower: "
                    "区間距離が不正です");

      robot.getWheelMotorControllerInstance().stopBoth();

      return;
    }

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "区間距離=%.2f mm",
                      distance);

    // ゲート検索
    const Gate* gate = findGate(from, to);

    if(gate != nullptr) {
      Logger::info("RouteFollower: "
                   "ゲートを検出しました");

      runGateSegment(from, to, distance, rotatedAtSegmentStart);

      continue;
    }

    Logger::info("RouteFollower: 通常区間");

    straight(distance);
  }
}

double RouteFollower::directionToHeading(Direction direction) const
{
  switch(direction) {
    case Direction::RIGHT:
      return 0.0;

    case Direction::UP:
      return 90.0;

    case Direction::LEFT:
      return 180.0;

    case Direction::DOWN:
      return -90.0;
  }

  return 0.0;
}

double RouteFollower::calculateRotationAngle(Direction from, Direction to) const
{
  const double currentHeading = directionToHeading(from);
  const double targetHeading = directionToHeading(to);

  return AngleNormalizer::normalizeAngle(currentHeading - targetHeading);
}

double RouteFollower::calculateDistance(const RouteState& from, const RouteState& to) const
{
  const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);
  const EtRallyMap::Node toNode = map.getNode(to.x, to.y);

  // X方向
  if(from.y == to.y) {
    return std::abs(toNode.x - fromNode.x);
  }

  // Y方向
  if(from.x == to.x) {
    return std::abs(toNode.y - fromNode.y);
  }

  Logger::printfLog(Logger::ERROR,
                    "RouteFollower: "
                    "斜めの経路 "
                    "(%d,%d) -> (%d,%d)",
                    from.x, from.y, to.x, to.y);

  return 0.0;
}

void RouteFollower::rotate(double angle)
{
  if(std::abs(angle) <= ROTATION_TOLERANCE) {
    return;
  }

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  auto condition = std::make_unique<RelativeAngleCondition>(robot, angle, ROTATION_TOLERANCE);

  RelativeRotation rotation(robot, std::move(condition), rotationPid, angle);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "回頭開始 %.2f 度",
                    angle);

  rotation.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "回頭完了 %.2f 度",
                    angle);
}

void RouteFollower::rotateForSquare(double angle)
{
  if(std::abs(angle) <= ROTATION_TOLERANCE) {
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "正方形回頭をスキップ %.2f 度",
                      angle);

    return;
  }

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  auto condition = std::make_unique<RelativeAngleCondition>(robot, angle, ROTATION_TOLERANCE);

  RelativeRotation rotation(robot, std::move(condition), squareRotationPid, angle);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "正方形回頭開始 %.2f 度",
                    angle);

  rotation.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "正方形回頭完了 %.2f 度",
                    angle);
}

void RouteFollower::straight(double distance)
{
  if(distance <= 0.0) {
    return;
  }

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  auto condition = std::make_unique<DistanceCondition>(robot, distance);

  Straight straightMotion(robot, std::move(condition), targetSpeed, straightAnglePid, true,
                          straightDeadbandRate, straightMaxoutRate);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "直進開始 %.2f mm",
                    distance);

  straightMotion.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "直進完了 %.2f mm",
                    distance);
}

void RouteFollower::backward(double distance)
{
  if(distance <= 0.0) {
    return;
  }

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  auto condition = std::make_unique<DistanceCondition>(robot, distance);

  // 目標速度を負にすることで後退する。
  Straight straightMotion(robot, std::move(condition), -std::abs(targetSpeed), straightAnglePid,
                          true, straightDeadbandRate, straightMaxoutRate);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "後退開始 %.2f mm",
                    distance);

  straightMotion.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "後退完了 %.2f mm",
                    distance);
}

bool RouteFollower::detectSquare(SquareAngleAdjustment::Result& result)
{
  Logger::info("RouteFollower: "
               "detectSquare 呼び出し");

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  // 正方形検出要求
  CameraServer::SquareDetectorRequest request{};

  request.roi.x = 0;
  request.roi.y = 0;
  request.roi.width = CAM_MAX_WIDTH;
  request.roi.height = CAM_MAX_HEIGHT;

  SquareAngleAdjustment adjustment(robot);

  result = adjustment.calculate(request);

  if(!result.wasDetected) {
    Logger::warning("RouteFollower: "
                    "Square detection FAILED");

    return false;
  }

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "正方形中心=(%.2f, %.2f)",
                    result.centerX, result.centerY);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "前方=%.2f "
                    "横方向=%.2f",
                    result.forwardDistance, result.lateralDistance);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "角度=%.2f "
                    "距離=%.2f",
                    result.correctionAngle, result.straightDistance);

  return true;
}

void RouteFollower::runGateSegment(const RouteState& from, const RouteState& to, double distance,
                                   bool rotatedAtSegmentStart)
{
  Logger::info("RouteFollower: "
               "runGateSegment CALLED");

  const Gate* gate = findGate(from, to);

  if(gate == nullptr) {
    straight(distance);

    return;
  }

  // 外周ゲート
  if(isOuterGate(*gate)) {
    Logger::info("RouteFollower: 外周ゲート");

    // ゲート中心までの距離
    const double distanceToGate = calculateDistanceToGate(from, *gate);

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "外周ゲートまでの距離=%.2f mm",
                      distanceToGate);

    if(distanceToGate < 0.0 || distanceToGate > distance) {
      Logger::warning("RouteFollower: "
                      "外周ゲート位置が不正です");

      straight(distance);

      return;
    }

    // QR①の位置
    double distanceToQr1 = distanceToGate - QR_TO_GATE_DISTANCE;

    if(distanceToQr1 < 0.0) {
      distanceToQr1 = 0.0;
    }

    // QR①の250mm手前で検出する。
    const double distanceToFirstDetection = distanceToQr1 - SQUARE_DETECTION_DISTANCE;

    // QR①補正スキップ判定
    const bool skipFirstCorrection = distanceToFirstDetection < -FIRST_DETECTION_TOLERANCE;

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "外周ゲート "
                      "区間開始時に回頭=%d "
                      "最初の検出までの距離=%.2f "
                      "許容誤差=%.2f "
                      "最初の補正をスキップ=%d",
                      rotatedAtSegmentStart ? 1 : 0, distanceToFirstDetection,
                      FIRST_DETECTION_TOLERANCE, skipFirstCorrection ? 1 : 0);

    // 外周ゲート QR①補正
    if(!skipFirstCorrection) {
      double firstMoveDistance = distanceToFirstDetection;

      // 少し通り過ぎているだけなら、
      // 後退せずその場でQR①を検出する。
      if(firstMoveDistance < 0.0) {
        Logger::printfLog(Logger::INFO,
                          "RouteFollower: "
                          "QR①検出予定位置を %.2f mm "
                          "通過していますが"
                          "許容範囲内なのでその場で検出します",
                          -firstMoveDistance);

        firstMoveDistance = 0.0;
      }

      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "外周ゲート "
                        "QR①検出位置まで移動 %.2f mm",
                        firstMoveDistance);

      straight(firstMoveDistance);

      // QR①検出
      Logger::info("RouteFollower: "
                   "外周ゲート QR①");

      SquareAngleAdjustment::Result firstResult{};

      const bool firstDetected = detectSquare(firstResult);

      // QR①検出成功
      if(firstDetected) {
        const double firstAngle = firstResult.correctionAngle;

        Logger::printfLog(Logger::INFO,
                          "RouteFollower: "
                          "外周ゲート "
                          "QR①補正角度=%.2f 度",
                          firstAngle);

        // QR①方向へ回頭
        rotateForSquare(firstAngle);

        Logger::printfLog(Logger::INFO,
                          "RouteFollower: "
                          "外周ゲート "
                          "QR①まで移動 %.2f mm",
                          firstResult.straightDistance);

        // QR①まで進む
        straight(firstResult.straightDistance);

        Logger::printfLog(Logger::INFO,
                          "RouteFollower: "
                          "外周ゲート "
                          "QR①補正角度を戻す %.2f 度",
                          -firstAngle);

        // 元の進行方向へ戻す
        rotateForSquare(-firstAngle);
      }

      // QR①検出失敗
      else {
        Logger::warning("RouteFollower: "
                        "外周ゲート "
                        "QR①の検出に失敗しました "
                        "→ 250mm直進");

        // QR①の想定位置まで進む
        straight(SQUARE_DETECTION_DISTANCE);
      }
    }

    // =========================================================
    // QR①補正をスキップする場合
    // =========================================================

    else {
      Logger::warning("RouteFollower: "
                      "外周ゲート "
                      "QR①補正をスキップ");

      /*
       * QR①補正を行えない場合でも、
       * QR①の想定位置まで移動して
       * そこを外周ゲート走行の基準位置とする。
       */
      if(distanceToQr1 > 0.0) {
        Logger::printfLog(Logger::INFO,
                          "RouteFollower: "
                          "外周ゲート "
                          "QR①想定位置まで移動 %.2f mm",
                          distanceToQr1);

        straight(distanceToQr1);
      }
    }

    // 外周ゲート通過
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR①位置から %.2f mm 前進",
                      OUTER_GATE_FORWARD_DISTANCE);

    /*
     * QR①位置から400mm前進する。
     * QR①はゲート中央の125mm手前にあるため、
     * 400mm前進することでゲートを通過する。
     */
    straight(OUTER_GATE_FORWARD_DISTANCE);

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "ゲート通過後 %.2f mm 後退",
                      OUTER_GATE_FORWARD_DISTANCE);

    /*
     * 向きを変えずに同じ距離を後退し、
     * QR①位置まで戻る。
     */
    backward(OUTER_GATE_FORWARD_DISTANCE);

    Logger::info("RouteFollower: "
                 "外周ゲート区間走行完了");

    /*
     * 外周ゲートではQR②補正を行わないため、
     * ここで処理を終了する。
     */
    return;
  }

  // 内側ゲート
  Logger::info("RouteFollower: 内側ゲート");

  /**
   * @brief この区間ですでに走行した距離[mm]
   *
   * この値を使用して残り距離を求める。
   */
  double traveledDistance = 0.0;

  /**
   * @brief QR②補正が成功したかどうか
   */
  bool secondCorrectionSucceeded = false;

  // 区間距離を超えないように走行する処理
  auto straightWithinSegment = [&](double requestedDistance) {
    if(requestedDistance <= 0.0) {
      return;
    }

    const double remaining = distance - traveledDistance;

    if(remaining <= 0.0) {
      Logger::warning("RouteFollower: "
                      "区間距離に到達済みのため"
                      "直進をスキップ");

      return;
    }

    double actualDistance = requestedDistance;

    // 要求距離が区間残距離を超える場合は制限する。
    if(actualDistance > remaining) {
      Logger::printfLog(Logger::WARNING,
                        "RouteFollower: "
                        "要求距離 %.2f mm を"
                        "残り距離 %.2f mm に制限",
                        actualDistance, remaining);

      actualDistance = remaining;
    }

    straight(actualDistance);

    traveledDistance += actualDistance;

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "区間走行済み=%.2f / %.2f mm",
                      traveledDistance, distance);
  };

  // ゲート中心までの距離
  const double distanceToGate = calculateDistanceToGate(from, *gate);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "ゲートまでの距離=%.2f mm",
                    distanceToGate);

  if(distanceToGate < 0.0 || distanceToGate > distance) {
    Logger::warning("RouteFollower: "
                    "ゲート位置が不正です");

    straight(distance);

    return;
  }

  // QR①の位置
  double distanceToQr1 = distanceToGate - QR_TO_GATE_DISTANCE;

  if(distanceToQr1 < 0.0) {
    distanceToQr1 = 0.0;
  }

  // QR①の250mm手前で最初の検出を行う。
  const double distanceToFirstDetection = distanceToQr1 - SQUARE_DETECTION_DISTANCE;

  // QR①補正スキップ判定
  const bool skipFirstCorrection = distanceToFirstDetection < -FIRST_DETECTION_TOLERANCE;

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "区間開始時に回頭=%d "
                    "最初の検出までの距離=%.2f "
                    "許容誤差=%.2f "
                    "最初の補正をスキップ=%d",
                    rotatedAtSegmentStart ? 1 : 0, distanceToFirstDetection,
                    FIRST_DETECTION_TOLERANCE, skipFirstCorrection ? 1 : 0);

  // QR①補正
  if(!skipFirstCorrection) {
    double firstMoveDistance = distanceToFirstDetection;

    // 少し通り過ぎているだけなら、
    // 後退せずその場でQR①を検出する。
    if(firstMoveDistance < 0.0) {
      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "QR①検出予定位置を %.2f mm "
                        "通過していますが"
                        "許容範囲内なのでその場で検出します",
                        -firstMoveDistance);

      firstMoveDistance = 0.0;
    }

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR①検出位置まで移動 %.2f mm",
                      firstMoveDistance);

    straightWithinSegment(firstMoveDistance);

    // QR①検出
    Logger::info("RouteFollower: QR①");

    SquareAngleAdjustment::Result firstResult{};

    const bool firstDetected = detectSquare(firstResult);

    // QR①検出成功
    if(firstDetected) {
      const double firstAngle = firstResult.correctionAngle;

      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "QR①補正角度=%.2f 度",
                        firstAngle);

      // QR①方向へ回頭
      rotateForSquare(firstAngle);

      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "QR①まで移動 %.2f mm",
                        firstResult.straightDistance);

      // QR①まで進む
      straightWithinSegment(firstResult.straightDistance);

      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "QR①補正角度を戻す %.2f 度",
                        -firstAngle);

      // 元の進行方向へ戻す
      rotateForSquare(-firstAngle);
    }

    // QR①検出失敗
    else {
      Logger::warning("RouteFollower: "
                      "QR①の検出に失敗しました "
                      "→ 250mm直進");

      straightWithinSegment(SQUARE_DETECTION_DISTANCE);
    }

    Logger::info("RouteFollower: "
                 "QR①位置 = QR②検出位置 "
                 "→ 追加直進をスキップ");
  }

  // =========================================================
  // QR①補正をスキップする場合
  // =========================================================

  else {
    Logger::warning("RouteFollower: "
                    "QR①補正をスキップ");

    // 区間開始位置からQR②検出位置までの距離を求める。
    double distanceToSecondDetection
        = distanceToGate + QR_TO_GATE_DISTANCE - SQUARE_DETECTION_DISTANCE;

    if(distanceToSecondDetection < 0.0) {
      distanceToSecondDetection = 0.0;
    }

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR②検出位置まで直接移動 %.2f mm",
                      distanceToSecondDetection);

    if(distanceToSecondDetection > 0.0) {
      straightWithinSegment(distanceToSecondDetection);
    }
  }

  // QR②補正
  Logger::info("RouteFollower: QR②");

  SquareAngleAdjustment::Result secondResult{};

  const bool secondDetected = detectSquare(secondResult);

  // QR②検出成功
  if(secondDetected) {
    const double secondAngle = secondResult.correctionAngle;

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR②補正角度=%.2f 度",
                      secondAngle);

    // QR②方向へ回頭
    rotateForSquare(secondAngle);

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR②まで移動 %.2f mm",
                      secondResult.straightDistance);

    // QR②まで進む
    straightWithinSegment(secondResult.straightDistance);

    /*
     * QR②位置まで補正できたため、
     * QR②の既知位置から残り距離を計算する。
     */
    secondCorrectionSucceeded = true;

    Logger::info("RouteFollower: "
                 "QR②補正成功 "
                 "→ 以降はQR②位置を基準に残距離を計算");
  }

  // QR②検出失敗
  else {
    Logger::warning("RouteFollower: "
                    "QR②の検出に失敗しました "
                    "→ 回頭なし");

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR②検出失敗時の直進 %.2f mm",
                      SQUARE_DETECTION_DISTANCE);

    straightWithinSegment(SQUARE_DETECTION_DISTANCE);
  }

  // 区間終端まで残りを走行
  double remainingDistance = 0.0;

  if(secondCorrectionSucceeded) {
    // QR②補正成功
    const double qr2PositionFromStart = distanceToGate + QR_TO_GATE_DISTANCE;

    remainingDistance = distance - qr2PositionFromStart;

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR②補正成功後 "
                      "区間距離=%.2f mm "
                      "QR②位置=%.2f mm "
                      "残り距離=%.2f mm",
                      distance, qr2PositionFromStart, remainingDistance);
  }

  else {
    // QR②補正なし
    // 残り距離を求める。
    remainingDistance = distance - traveledDistance;

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR②補正なし "
                      "走行済み距離=%.2f mm "
                      "残り距離=%.2f mm",
                      traveledDistance, remainingDistance);
  }

  if(remainingDistance < 0.0) {
    remainingDistance = 0.0;
  }

  // 最後の通常直進
  if(remainingDistance > 0.0) {
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "補正後の通常直進 %.2f mm",
                      remainingDistance);

    straight(remainingDistance);
  }

  Logger::info("RouteFollower: "
               "ゲート区間走行完了");
}

const Gate* RouteFollower::findGate(const RouteState& from, const RouteState& to) const
{
  for(const Gate& gate : robot.getMapData().getGates()) {
    const std::vector<GatePass> passes = robot.getMapData().getGatePasses(gate.color);

    for(const GatePass& pass : passes) {
      // Y方向
      if(from.x == to.x && pass.entrance.x == from.x && pass.exit.x == from.x) {
        // Y増加
        if(to.y > from.y) {
          const bool entranceInside = pass.entrance.y >= from.y && pass.entrance.y <= to.y;
          const bool exitInside = pass.exit.y >= from.y && pass.exit.y <= to.y;
          const bool correctDirection = pass.exit.y > pass.entrance.y;

          if(entranceInside && exitInside && correctDirection) {
            return &gate;
          }
        }

        // Y減少
        if(to.y < from.y) {
          const bool entranceInside = pass.entrance.y <= from.y && pass.entrance.y >= to.y;
          const bool exitInside = pass.exit.y <= from.y && pass.exit.y >= to.y;
          const bool correctDirection = pass.exit.y < pass.entrance.y;

          if(entranceInside && exitInside && correctDirection) {
            return &gate;
          }
        }
      }

      // X方向
      if(from.y == to.y && pass.entrance.y == from.y && pass.exit.y == from.y) {
        // X増加
        if(to.x > from.x) {
          const bool entranceInside = pass.entrance.x >= from.x && pass.entrance.x <= to.x;
          const bool exitInside = pass.exit.x >= from.x && pass.exit.x <= to.x;
          const bool correctDirection = pass.exit.x > pass.entrance.x;

          if(entranceInside && exitInside && correctDirection) {
            return &gate;
          }
        }

        // X減少
        if(to.x < from.x) {
          const bool entranceInside = pass.entrance.x <= from.x && pass.entrance.x >= to.x;
          const bool exitInside = pass.exit.x <= from.x && pass.exit.x >= to.x;
          const bool correctDirection = pass.exit.x < pass.entrance.x;

          if(entranceInside && exitInside && correctDirection) {
            return &gate;
          }
        }
      }
    }
  }

  return nullptr;
}

bool RouteFollower::isOuterGate(const Gate& gate) const
{
  // 横向きゲート
  if(gate.start.y == gate.end.y) {
    const int gateY = gate.start.y;

    /*
     * 上端または下端の外周ゲート。
     * 走行可能なグリッド座標は偶数で、
     * ゲートはその間の奇数座標に存在するため、
     * 外周ゲートは1または最大値-1になる。
     */
    return gateY == 1 || gateY == SystemInfo::Y_GRID_NUM - 1;
  }

  // 縦向きゲート
  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;

    /*
     * 左端または右端の外周ゲート。
     * 走行可能なグリッド座標は偶数で、
     * ゲートはその間の奇数座標に存在するため、
     * 外周ゲートは1または最大値-1になる。
     */
    return gateX == 1 || gateX == SystemInfo::X_GRID_NUM - 1;
  }

  return false;
}

double RouteFollower::calculateDistanceToGate(const RouteState& from, const Gate& gate) const
{
  const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);

  // 横向きゲート
  if(gate.start.y == gate.end.y) {
    const int centerX = (gate.start.x + gate.end.x) / 2;
    const int gateY = gate.start.y;
    const EtRallyMap::Node gateNode = map.getNode(centerX, gateY);

    /*
     * 横向きゲートはY方向に通過するため、
     * Y座標差を距離として使用する。
     */
    return std::abs(gateNode.y - fromNode.y);
  }

  // 縦向きゲート
  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;
    const int centerY = (gate.start.y + gate.end.y) / 2;
    const EtRallyMap::Node gateNode = map.getNode(gateX, centerY);

    /*
     * 縦向きゲートはX方向に通過するため、
     * X座標差を距離として使用する。
     */
    return std::abs(gateNode.x - fromNode.x);
  }

  Logger::error("RouteFollower: "
                "不正なゲート");

  return -1.0;
}