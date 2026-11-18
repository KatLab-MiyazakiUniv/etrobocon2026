
/**
 * @file   RouteFollower.cpp
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214 okuyama0528
 */

#include "RouteFollower.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

namespace {
  /**
   * @brief 90度回頭に対して指定する回頭角度[deg]
   */
  constexpr double ROTATION_ANGLE = 90.0;

  /**
   * @brief 回頭終了判定許容誤差[deg]
   */
  constexpr double ROTATION_TOLERANCE = 3.0;

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
   * @brief QR検出失敗時に直進する距離[mm]
   */
  constexpr double SQUARE_DETECTION_FAILURE_DISTANCE = 125.0;

  /**
   * @brief QR①検出位置の許容誤差[mm]
   */
  constexpr double FIRST_DETECTION_TOLERANCE = 15.0;

  /**
   * @brief 外周ゲートでQR①位置から前進する距離[mm]
   */
  constexpr double OUTER_GATE_FORWARD_DISTANCE = 400.0;

  /**
   * @brief 回頭後補正とゲート前補正を分離する基準距離[mm]
   */
  constexpr double TURN_CORRECTION_MIN_DISTANCE = 250.0;

  /**
   * @brief 動作切り替え待機時間[ms]
   */
  constexpr int MOTION_SWITCH_WAIT = 100;

  /**
   * @brief IMUリセット前後の待機時間[ms]
   */
  constexpr int IMU_RESET_WAIT = 200;

  /**
   * @brief RouteFollowerの走行可能区間が残っているか判定する継続条件
   */
  class RouteContinuationCondition : public BaseContinuationCondition {
   public:
    RouteContinuationCondition(Robot& _robot, std::function<bool()> _shouldContinue)
      : BaseContinuationCondition(_robot), shouldContinueCallback(std::move(_shouldContinue))
    {
    }

    bool shouldContinue() override { return shouldContinueCallback(); }

   private:
    std::function<bool()> shouldContinueCallback;
  };

}  // namespace

RouteFollower::RouteFollower(Robot& _robot, const EtRallyMap& _map, double _targetSpeed,
                             const Pid::PidGain& _rotationPid,
                             const Pid::PidGain& _squareRotationPid,
                             const Pid::PidGain& _straightAnglePid, double _straightDeadbandRate,
                             double _straightMaxoutRate)
  : BaseMotion(_robot, std::make_unique<RouteContinuationCondition>(
                           _robot, [this]() { return !failed && nextIndex < route.size(); })),
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

void RouteFollower::setRoute(const std::vector<RouteState>& _route)
{
  route = _route;
}

void RouteFollower::prepare()
{
  nextIndex = 1;
  failed = false;

  if(route.size() < 2) {
    Logger::warning("RouteFollower: 経路が2点未満のため走行できません");
  }
}

/**
 * @brief 経路の1区間を走行する
 */
void RouteFollower::executeStep()
{
  const RouteState& from = route[nextIndex - 1];
  const RouteState& to = route[nextIndex];

  Logger::printfLog(Logger::INFO,
                    "RouteFollower[%d]: "
                    "現在走行する区間 "
                    "(%d,%d) → (%d,%d)",
                    static_cast<int>(nextIndex), from.x, from.y, to.x, to.y);

  // 次の区間を走行するために必要な回頭角度を計算する。
  const double rotationAngle = calculateRotationAngle(from.direction, to.direction);

  // 現在の区間開始時に回頭が必要か判定する。
  const bool rotatedNow = std::abs(rotationAngle) > ROTATION_TOLERANCE;

  // 直前の区間が同一座標での回頭だったか判定する。
  bool rotatedPreviously = false;

  if(nextIndex >= 2) {
    const RouteState& previous = route[nextIndex - 2];

    rotatedPreviously
        = previous.x == from.x && previous.y == from.y && previous.direction != from.direction;
  }

  // 現在または直前の区間で回頭したか判定する。
  const bool rotatedAtSegmentStart = rotatedNow || rotatedPreviously;

  if(rotatedNow) {
    Logger::printfLog(Logger::INFO, "RouteFollower: 回頭開始 %.2f deg", rotationAngle);

    rotate(rotationAngle);
  }

  // 同一座標の場合は回頭のみ行う。
  // 回頭後のQR補正は次の直進区間で行う。
  if(from.x == to.x && from.y == to.y) {
    Logger::info("RouteFollower: 回頭のみの区間");

    ++nextIndex;
    return;
  }

  // 区間距離を計算する。
  const double distance = calculateDistance(from, to);

  if(distance <= 0.0) {
    Logger::error("RouteFollower: 区間距離が不正です");

    robot.getWheelMotorControllerInstance().stopBoth();

    failed = true;
    return;
  }

  Logger::printfLog(Logger::INFO, "RouteFollower: 区間距離=%.2f mm", distance);

  // 外周上の区間か判定する。
  const bool isOuterSegment = from.x == 0 || from.x == SystemInfo::X_GRID_NUM || from.y == 0
                              || from.y == SystemInfo::Y_GRID_NUM;

  const Gate* gate = findGate(from, to);

  // ゲート区間では専用の処理を行う。
  if(gate != nullptr) {
    Logger::info("RouteFollower: ゲートを検出しました");

    runGateSegment(from, to, distance, rotatedAtSegmentStart);

    ++nextIndex;
    return;
  }

  Logger::info("RouteFollower: 通常区間");

  // 通常区間での走行済み距離
  double traveledDistance = 0.0;

  // 外周では回頭後補正を行わない。
  if(isOuterSegment) {
    if(rotatedAtSegmentStart) {
      Logger::info("RouteFollower: 外周のため回頭後補正をスキップ");
    }
  }
  // 内側で回頭した場合はQR補正を行う。
  else if(rotatedAtSegmentStart) {
    Logger::info("RouteFollower: 内側で回頭したためQR補正開始");

    SquareAngleAdjustment::Result result{};

    const bool detected = detectSquare(result);

    if(detected) {
      const double correctionAngle = result.correctionAngle;

      Logger::printfLog(Logger::INFO, "RouteFollower: 回頭後補正角度=%.2f 度", correctionAngle);

      // QR方向へ回頭
      rotateForSquare(correctionAngle);

      // 区間距離を超えないように直進する。
      const double correctionDistance = std::max(0.0, std::min(result.straightDistance, distance));

      Logger::printfLog(Logger::INFO, "RouteFollower: 回頭後QRまで移動 %.2f mm",
                        correctionDistance);

      straight(correctionDistance);

      traveledDistance += correctionDistance;

      // 元の進行方向へ戻す。
      rotateForSquare(-correctionAngle);

      Logger::info("RouteFollower: 回頭後QR補正完了");
    } else {
      Logger::warning("RouteFollower: 回頭後QR検出失敗 → 125mm直進");

      const double failureDistance = std::min(SQUARE_DETECTION_FAILURE_DISTANCE, distance);

      straight(failureDistance);

      traveledDistance += failureDistance;
    }
  }

  // 補正で進んだ距離を差し引く。
  double remainingDistance = distance - traveledDistance;

  if(remainingDistance < 0.0) {
    remainingDistance = 0.0;
  }

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "通常区間 残り距離=%.2f mm",
                    remainingDistance);

  // 区間終点まで直進する。
  straight(remainingDistance);

  ++nextIndex;
}

void RouteFollower::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
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

  const double angle = AngleNormalizer::normalizeAngle(currentHeading - targetHeading);

  // 90度を基準として指定回頭角度に変換する。
  return angle * (ROTATION_ANGLE / 90.0);
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

  const double currentAngle = robot.getIMUControllerInstance().getAzimuth();

  const double targetAbsAngle = AngleNormalizer::normalizeAngle(currentAngle + angle);

  auto condition
      = std::make_unique<AbsoluteAngleCondition>(robot, targetAbsAngle, ROTATION_TOLERANCE);

  AbsoluteRotation rotation(robot, std::move(condition), rotationPid, targetAbsAngle);

  Logger::printfLog(Logger::INFO, "RouteFollower: 回頭開始 %.2f 度", angle);

  rotation.run();

  Logger::printfLog(Logger::INFO, "RouteFollower: 回頭完了 %.2f 度", angle);
}

void RouteFollower::rotateForSquare(double angle)
{
  if(std::abs(angle) <= ROTATION_TOLERANCE) {
    Logger::printfLog(Logger::INFO, "RouteFollower: 正方形回頭をスキップ %.2f 度", angle);
    return;
  }

  robot.getWheelMotorControllerInstance().stopBoth();
  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  const double currentAngle = robot.getIMUControllerInstance().getAzimuth();

  const double targetAbsAngle = AngleNormalizer::normalizeAngle(currentAngle + angle);

  auto condition
      = std::make_unique<AbsoluteAngleCondition>(robot, targetAbsAngle, ROTATION_TOLERANCE);

  AbsoluteRotation rotation(robot, std::move(condition), squareRotationPid, targetAbsAngle);

  Logger::printfLog(Logger::INFO, "RouteFollower: 正方形回頭開始 %.2f 度", angle);

  rotation.run();

  Logger::printfLog(Logger::INFO, "RouteFollower: 正方形回頭完了 %.2f 度", angle);
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

  Logger::printfLog(Logger::INFO, "RouteFollower: 直進開始 %.2f mm", distance);

  straightMotion.run();

  Logger::printfLog(Logger::INFO, "RouteFollower: 直進完了 %.2f mm", distance);
}

void RouteFollower::backward(double distance)
{
  // バック走行は使用しない。
  Logger::printfLog(Logger::INFO, "RouteFollower: 後退処理をスキップ %.2f mm", distance);
}

bool RouteFollower::detectSquare(SquareAngleAdjustment::Result& result)
{
  Logger::info("RouteFollower: detectSquare 呼び出し");

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
    Logger::warning("RouteFollower: Square detection FAILED");
    return false;
  }

  Logger::printfLog(Logger::INFO, "RouteFollower: 正方形中心=(%.2f, %.2f)", result.centerX,
                    result.centerY);

  Logger::printfLog(Logger::INFO, "RouteFollower: 前方=%.2f 横方向=%.2f", result.forwardDistance,
                    result.lateralDistance);

  Logger::printfLog(Logger::INFO, "RouteFollower: 角度=%.2f 距離=%.2f", result.correctionAngle,
                    result.straightDistance);

  return true;
}

void RouteFollower::runGateSegment(const RouteState& from, const RouteState& to, double distance,
                                   bool rotatedAtSegmentStart)
{
  Logger::info("RouteFollower: runGateSegment CALLED");

  const Gate* gate = findGate(from, to);

  if(gate == nullptr) {
    straight(distance);
    return;
  }

  // =========================================================
  // 外周ゲート
  // =========================================================
  if(isOuterGate(*gate)) {
    Logger::info("RouteFollower: 外周ゲート");

    if(rotatedAtSegmentStart) {
      Logger::info("RouteFollower: 外周のため回頭後補正をスキップ");
    }

    // ゲート中心までの距離
    const double distanceToGate = calculateDistanceToGate(from, *gate);

    Logger::printfLog(Logger::INFO, "RouteFollower: 外周ゲートまでの距離=%.2f mm", distanceToGate);

    if(distanceToGate < 0.0 || distanceToGate > distance) {
      Logger::warning("RouteFollower: 外周ゲート位置が不正です");
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
                      "最初の検出までの距離=%.2f "
                      "最初の補正をスキップ=%d",
                      distanceToFirstDetection, skipFirstCorrection ? 1 : 0);

    // QR①想定位置までの走行距離
    double traveledDistance = 0.0;

    // =========================================================
    // 外周ゲート QR①補正
    // =========================================================
    if(!skipFirstCorrection) {
      double firstMoveDistance = distanceToFirstDetection;

      if(firstMoveDistance < 0.0) {
        Logger::printfLog(Logger::INFO,
                          "RouteFollower: "
                          "QR①検出予定位置を %.2f mm 通過 "
                          "→ その場で検出",
                          -firstMoveDistance);

        firstMoveDistance = 0.0;
      }

      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "外周ゲート QR①検出位置まで移動 %.2f mm",
                        firstMoveDistance);

      straight(firstMoveDistance);

      traveledDistance += firstMoveDistance;

      Logger::info("RouteFollower: 外周ゲート QR①");

      SquareAngleAdjustment::Result firstResult{};

      const bool firstDetected = detectSquare(firstResult);

      if(firstDetected) {
        const double firstAngle = firstResult.correctionAngle;

        Logger::printfLog(Logger::INFO,
                          "RouteFollower: "
                          "外周ゲート QR①補正角度=%.2f 度",
                          firstAngle);

        rotateForSquare(firstAngle);

        Logger::printfLog(Logger::INFO,
                          "RouteFollower: "
                          "外周ゲート QR①まで移動 %.2f mm",
                          firstResult.straightDistance);

        straight(firstResult.straightDistance);

        rotateForSquare(-firstAngle);

        // 補正成功時はQR①の既知位置を基準とする。
        traveledDistance = distanceToQr1;
      } else {
        Logger::warning("RouteFollower: "
                        "外周ゲート QR①検出失敗 → 125mm直進");

        straight(SQUARE_DETECTION_FAILURE_DISTANCE);

        traveledDistance += SQUARE_DETECTION_FAILURE_DISTANCE;

        // QR①想定位置までの残りを直進する。
        const double remainingToQr1 = std::max(0.0, distanceToQr1 - traveledDistance);

        if(remainingToQr1 > 0.0) {
          Logger::printfLog(Logger::INFO,
                            "RouteFollower: "
                            "外周ゲート QR①想定位置まで残り %.2f mm",
                            remainingToQr1);

          straight(remainingToQr1);

          traveledDistance += remainingToQr1;
        }
      }
    } else {
      Logger::warning("RouteFollower: 外周ゲート QR①補正をスキップ");

      // QR①の想定位置まで移動する。
      if(distanceToQr1 > 0.0) {
        Logger::printfLog(Logger::INFO,
                          "RouteFollower: "
                          "外周ゲート QR①想定位置まで移動 %.2f mm",
                          distanceToQr1);

        straight(distanceToQr1);
      }

      traveledDistance = distanceToQr1;
    }

    // =========================================================
    // 外周ゲート通過
    // =========================================================
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR①位置から %.2f mm 前進",
                      OUTER_GATE_FORWARD_DISTANCE);

    straight(OUTER_GATE_FORWARD_DISTANCE);

    traveledDistance += OUTER_GATE_FORWARD_DISTANCE;

    // =========================================================
    // バックせず区間終点まで前進
    // =========================================================
    double remainingDistance = distance - traveledDistance;

    if(remainingDistance < 0.0) {
      remainingDistance = 0.0;
    }

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "外周ゲート通過後 残り距離=%.2f mm",
                      remainingDistance);

    // 後退はせず、そのまま前進する。
    straight(remainingDistance);

    Logger::info("RouteFollower: 外周ゲート区間走行完了");

    return;
  }

  // =========================================================
  // 内側ゲート
  // =========================================================
  Logger::info("RouteFollower: 内側ゲート");

  /**
   * @brief この区間ですでに走行した距離[mm]
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
      Logger::warning("RouteFollower: 区間距離に到達済みのため直進をスキップ");
      return;
    }

    double actualDistance = requestedDistance;

    if(actualDistance > remaining) {
      Logger::printfLog(Logger::WARNING,
                        "RouteFollower: "
                        "要求距離 %.2f mm を残り距離 %.2f mm に制限",
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

  Logger::printfLog(Logger::INFO, "RouteFollower: ゲートまでの距離=%.2f mm", distanceToGate);

  if(distanceToGate < 0.0 || distanceToGate > distance) {
    Logger::warning("RouteFollower: ゲート位置が不正です");

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
                    "最初の補正をスキップ=%d",
                    rotatedAtSegmentStart ? 1 : 0, distanceToFirstDetection,
                    skipFirstCorrection ? 1 : 0);

  // =========================================================
  // 内側ゲートへ向かう途中の回頭後補正
  // =========================================================
  if(rotatedAtSegmentStart) {
    // ゲート前QR補正まで十分な距離がある場合のみ実施する。
    const bool shouldCorrectAfterTurn = distanceToFirstDetection >= TURN_CORRECTION_MIN_DISTANCE;

    if(shouldCorrectAfterTurn) {
      Logger::info("RouteFollower: "
                   "ゲートまで距離があるため回頭後補正を実施");

      SquareAngleAdjustment::Result turnResult{};

      const bool turnDetected = detectSquare(turnResult);

      if(turnDetected) {
        const double turnAngle = turnResult.correctionAngle;

        Logger::printfLog(Logger::INFO,
                          "RouteFollower: "
                          "回頭後補正角度=%.2f 度",
                          turnAngle);

        // QR方向へ回頭
        rotateForSquare(turnAngle);

        // QRまで直進
        Logger::printfLog(Logger::INFO,
                          "RouteFollower: "
                          "回頭後QRまで移動 %.2f mm",
                          turnResult.straightDistance);

        straightWithinSegment(turnResult.straightDistance);

        // 元の進行方向へ戻す。
        rotateForSquare(-turnAngle);

        Logger::info("RouteFollower: 回頭後補正完了");
      } else {
        Logger::warning("RouteFollower: "
                        "回頭後QR検出失敗 → 125mm直進");

        straightWithinSegment(SQUARE_DETECTION_FAILURE_DISTANCE);
      }
    } else {
      Logger::info("RouteFollower: "
                   "すぐにゲート前補正を行うため回頭後補正をスキップ");
    }
  }

  // =========================================================
  // QR①補正
  // =========================================================
  if(!skipFirstCorrection) {
    // 回頭後補正ですでに進んだ距離を差し引く。
    double firstMoveDistance = distanceToFirstDetection - traveledDistance;

    if(firstMoveDistance < 0.0) {
      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "QR①検出位置を %.2f mm 通過 "
                        "→ その場で検出",
                        -firstMoveDistance);

      firstMoveDistance = 0.0;
    }

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR①検出位置まで移動 %.2f mm",
                      firstMoveDistance);

    straightWithinSegment(firstMoveDistance);

    Logger::info("RouteFollower: QR①");

    SquareAngleAdjustment::Result firstResult{};

    const bool firstDetected = detectSquare(firstResult);

    if(firstDetected) {
      const double firstAngle = firstResult.correctionAngle;

      Logger::printfLog(Logger::INFO, "RouteFollower: QR①補正角度=%.2f 度", firstAngle);

      // QR①方向へ回頭
      rotateForSquare(firstAngle);

      Logger::printfLog(Logger::INFO, "RouteFollower: QR①まで移動 %.2f mm",
                        firstResult.straightDistance);

      straightWithinSegment(firstResult.straightDistance);

      // 元の進行方向へ戻す。
      rotateForSquare(-firstAngle);
    } else {
      Logger::warning("RouteFollower: "
                      "QR①の検出に失敗しました → 125mm直進");

      straightWithinSegment(SQUARE_DETECTION_FAILURE_DISTANCE);
    }

    Logger::info("RouteFollower: "
                 "QR①位置 = QR②検出位置 "
                 "→ 追加直進をスキップ");
  }

  // =========================================================
  // QR①補正をスキップする場合
  // =========================================================
  else {
    Logger::warning("RouteFollower: QR①補正をスキップ");

    // 区間開始位置からQR②検出位置までの距離
    double distanceToSecondDetection
        = distanceToGate + QR_TO_GATE_DISTANCE - SQUARE_DETECTION_DISTANCE;

    // 回頭後補正で移動した距離を差し引く。
    distanceToSecondDetection -= traveledDistance;

    if(distanceToSecondDetection < 0.0) {
      distanceToSecondDetection = 0.0;
    }

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR②検出位置まで直接移動 %.2f mm",
                      distanceToSecondDetection);

    straightWithinSegment(distanceToSecondDetection);
  }

  // =========================================================
  // QR②補正
  // =========================================================
  Logger::info("RouteFollower: QR②");

  SquareAngleAdjustment::Result secondResult{};

  const bool secondDetected = detectSquare(secondResult);

  if(secondDetected) {
    const double secondAngle = secondResult.correctionAngle;

    Logger::printfLog(Logger::INFO, "RouteFollower: QR②補正角度=%.2f 度", secondAngle);

    // QR②方向へ回頭
    rotateForSquare(secondAngle);

    Logger::printfLog(Logger::INFO, "RouteFollower: QR②まで移動 %.2f mm",
                      secondResult.straightDistance);

    // QR②まで進む
    straightWithinSegment(secondResult.straightDistance);

    // QR②補正後に停止
    robot.getWheelMotorControllerInstance().stopBoth();

    ClockUtil::sleep(IMU_RESET_WAIT);

    // QR②補正完了後にIMUの角度をリセット
    robot.getIMUControllerInstance().resetAzimuth();

    ClockUtil::sleep(IMU_RESET_WAIT);

    Logger::printfLog(Logger::INFO, "RouteFollower: QR②補正後IMUリセット=%.2f 度",
                      robot.getIMUControllerInstance().getAzimuth());

    secondCorrectionSucceeded = true;

    Logger::info("RouteFollower: "
                 "QR②補正成功 "
                 "→ 以降はQR②位置を基準に残距離を計算");
  } else {
    Logger::warning("RouteFollower: "
                    "QR②の検出に失敗しました → 回頭なし");

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR②検出失敗時の直進 %.2f mm",
                      SQUARE_DETECTION_FAILURE_DISTANCE);

    straightWithinSegment(SQUARE_DETECTION_FAILURE_DISTANCE);
  }

  // =========================================================
  // 区間終端まで残りを走行
  // =========================================================
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
  } else {
    // QR②補正なし
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

  if(remainingDistance > 0.0) {
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "補正後の通常直進 %.2f mm",
                      remainingDistance);

    straight(remainingDistance);
  }

  Logger::info("RouteFollower: ゲート区間走行完了");
}

const Gate* RouteFollower::findGate(const RouteState& from, const RouteState& to) const
{
  for(const Gate& gate : robot.getMapDataInstance().getGates()) {
    const std::vector<GatePass> passes = robot.getMapDataInstance().getGatePasses(gate.color);

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
     * ゲートは奇数座標に存在する。
     */
    return gateY == 1 || gateY == SystemInfo::Y_GRID_NUM - 1;
  }

  // 縦向きゲート
  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;

    /*
     * 左端または右端の外周ゲート。
     * ゲートは奇数座標に存在する。
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

  Logger::error("RouteFollower: 不正なゲート");

  return -1.0;
}
