/**
 * @file   RouteFollower.cpp
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#include "RouteFollower.h"

#include <cmath>
#include <memory>

#include "AngleNormalizer.h"
#include "ClockUtil.h"
#include "DistanceCondition.h"
#include "Logger.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "SocketProtocol.h"
#include "Straight.h"
#include "SystemInfo.h"

namespace {

  /**
   * @brief 回頭終了判定許容誤差[deg]
   */
  constexpr double ROTATION_TOLERANCE = 0.5;

  /**
   * @brief QR①からQR②までの距離[mm]
   *
   * QR①
   *  |
   *  | 125mm
   *  |
   * ゲート
   *  |
   *  | 125mm
   *  |
   * QR②
   */
  constexpr double QR_DISTANCE = 250.0;

  /**
   * @brief QRとゲート中央の距離[mm]
   */
  constexpr double QR_TO_GATE_DISTANCE = QR_DISTANCE / 2.0;

  /**
   * @brief QRを検出する基準距離[mm]
   *
   * QRの250mm手前から検出する。
   *
   * QR①からQR②までの距離も250mmなので、
   * QR①到達位置がそのままQR②検出位置になる。
   */
  constexpr double SQUARE_DETECTION_DISTANCE = 250.0;

  /**
   * @brief 動作切り替え待機時間[ms]
   */
  constexpr int MOTION_SWITCH_WAIT = 200;

}  // namespace

RouteFollower::RouteFollower(Robot& _robot, const EtRallyMap& _map, const MapData& _mapData,
                             double _targetSpeed, const Pid::PidGain& _rotationPid,
                             const Pid::PidGain& _squareRotationPid, const Pid::PidGain& _rightPid,
                             const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid)
  : robot(_robot),
    map(_map),
    mapData(_mapData),
    targetSpeed(_targetSpeed),
    rotationPid(_rotationPid),
    squareRotationPid(_squareRotationPid),
    rightPid(_rightPid),
    leftPid(_leftPid),
    straightAnglePid(_straightAnglePid)
{
  LOG_CREATE("RouteFollower");
}

void RouteFollower::run(const std::vector<RouteState>& route)
{
  if(route.size() < 2) {
    Logger::warning("RouteFollower: route size < 2");

    return;
  }

  for(std::size_t i = 1; i < route.size(); ++i) {
    const RouteState& from = route[i - 1];

    const RouteState& to = route[i];

    Logger::printfLog(Logger::INFO,
                      "RouteFollower[%d]: "
                      "(%d,%d) -> (%d,%d)",
                      static_cast<int>(i), from.x, from.y, to.x, to.y);

    // =====================================================
    // 1. 通常回頭
    // =====================================================

    const double rotationAngle = calculateRotationAngle(from.direction, to.direction);

    /*
     * この区間開始時に回頭したかを保存する。
     *
     * ゲート直前回頭判定に使用する。
     */
    const bool rotatedAtSegmentStart = std::abs(rotationAngle) > ROTATION_TOLERANCE;

    if(rotatedAtSegmentStart) {
      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "rotation required %.2f deg",
                        rotationAngle);

      rotate(rotationAngle);
    }

    // =====================================================
    // 2. 同一座標
    //
    // 回頭だけを表すRouteStateの場合
    // =====================================================

    if(from.x == to.x && from.y == to.y) {
      continue;
    }

    // =====================================================
    // 3. 区間距離
    // =====================================================

    const double distance = calculateDistance(from, to);

    if(distance <= 0.0) {
      Logger::error("RouteFollower: invalid distance");

      robot.getWheelMotorControllerInstance().stopBoth();

      return;
    }

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "segment distance=%.2f mm",
                      distance);

    // =====================================================
    // 4. ゲート検索
    // =====================================================

    const Gate* gate = findGate(from, to);

    if(gate != nullptr) {
      Logger::info("RouteFollower: "
                   "===== GATE FOUND =====");

      runGateSegment(from, to, distance, rotatedAtSegmentStart);

      continue;
    }

    // =====================================================
    // 5. 通常区間
    // =====================================================

    Logger::info("RouteFollower: normal segment");

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
                    "diagonal route "
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
                    "Rotation START %.2f deg",
                    angle);

  rotation.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Rotation FINISHED %.2f deg",
                    angle);
}

void RouteFollower::rotateForSquare(double angle)
{
  if(std::abs(angle) <= ROTATION_TOLERANCE) {
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "Square Rotation skipped %.2f deg",
                      angle);

    return;
  }

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  auto condition = std::make_unique<RelativeAngleCondition>(robot, angle, ROTATION_TOLERANCE);

  RelativeRotation rotation(robot, std::move(condition), squareRotationPid, angle);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Square Rotation START %.2f deg",
                    angle);

  rotation.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Square Rotation FINISHED %.2f deg",
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

  Straight straightMotion(robot, std::move(condition), targetSpeed, rightPid, leftPid,
                          straightAnglePid, true);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Straight START %.2f mm",
                    distance);

  straightMotion.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Straight FINISHED %.2f mm",
                    distance);
}

bool RouteFollower::detectSquare(SquareAngleAdjustment::Result& result)
{
  Logger::info("RouteFollower: "
               "detectSquare CALLED");

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  // =====================================================
  // 正方形検出要求
  // =====================================================

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
                    "Square center=(%.2f, %.2f)",
                    result.centerX, result.centerY);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "forward=%.2f "
                    "lateral=%.2f",
                    result.forwardDistance, result.lateralDistance);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "angle=%.2f "
                    "distance=%.2f",
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

  // =====================================================
  // 外周ゲート
  // =====================================================

  if(isOuterGate(*gate)) {
    Logger::info("RouteFollower: OUTER GATE");

    straight(distance);

    return;
  }

  // =====================================================
  // 内側ゲート
  // =====================================================

  Logger::info("RouteFollower: INNER GATE");

  // =====================================================
  // ゲート中心までの距離
  // =====================================================

  const double distanceToGate = calculateDistanceToGate(from, *gate);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "distanceToGate=%.2f mm",
                    distanceToGate);

  if(distanceToGate < 0.0 || distanceToGate > distance) {
    Logger::warning("RouteFollower: "
                    "invalid gate position");

    straight(distance);

    return;
  }

  // =====================================================
  // QR①位置
  //
  // ゲート中心の125mm手前
  // =====================================================

  double distanceToQr1 = distanceToGate - QR_TO_GATE_DISTANCE;

  if(distanceToQr1 < 0.0) {
    distanceToQr1 = 0.0;
  }

  // =====================================================
  // QR①検出位置
  //
  // QR①の250mm手前
  //
  // QR①はゲート中心の125mm手前なので、
  //
  // 125 + 250 = 375mm
  //
  // ゲート中心の375mm手前で検出する。
  // =====================================================

  const double distanceToFirstDetection = distanceToQr1 - SQUARE_DETECTION_DISTANCE;

  // =====================================================
  // QR①補正スキップ判定
  // =====================================================

  const bool skipFirstCorrection = rotatedAtSegmentStart && distanceToFirstDetection <= 0.0;

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "rotatedAtSegmentStart=%d "
                    "distanceToFirstDetection=%.2f "
                    "skipFirstCorrection=%d",
                    rotatedAtSegmentStart ? 1 : 0, distanceToFirstDetection,
                    skipFirstCorrection ? 1 : 0);

  // =====================================================
  // QR①補正
  // =====================================================

  if(!skipFirstCorrection) {
    double firstMoveDistance = distanceToFirstDetection;

    if(firstMoveDistance < 0.0) {
      firstMoveDistance = 0.0;
    }

    // ---------------------------------------------------
    // QR①検出位置まで移動
    // ---------------------------------------------------

    if(firstMoveDistance > 0.0) {
      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "Move to QR1 detection point "
                        "%.2f mm",
                        firstMoveDistance);

      straight(firstMoveDistance);
    }

    // ---------------------------------------------------
    // QR①検出
    // ---------------------------------------------------

    Logger::info("RouteFollower: "
                 "========== QR1 ==========");

    SquareAngleAdjustment::Result firstResult{};

    const bool firstDetected = detectSquare(firstResult);

    // ===================================================
    // QR①検出成功
    // ===================================================

    if(firstDetected) {
      const double firstAngle = firstResult.correctionAngle;

      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "QR1 correction angle=%.2f deg",
                        firstAngle);

      // QR①方向へ回頭
      rotateForSquare(firstAngle);

      // QR①まで進む
      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "Move to QR1 %.2f mm",
                        firstResult.straightDistance);

      straight(firstResult.straightDistance);

      // =================================================
      // 元の進行方向へ戻す
      // =================================================

      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "Restore QR1 angle %.2f deg",
                        -firstAngle);

      rotateForSquare(-firstAngle);
    }

    // ===================================================
    // QR①検出失敗
    // ===================================================

    else {
      Logger::warning("RouteFollower: "
                      "QR1 detection failed "
                      "-> straight 250mm");

      /*
       * QR①の250mm手前なので、
       * 250mm通常直進してQR①位置まで進む。
       */
      straight(SQUARE_DETECTION_DISTANCE);
    }

    // ===================================================
    // QR①位置 = QR②検出位置
    //
    // QR①とQR②の距離が250mmで、
    // QR②を250mm手前から検出するため、
    // 追加直進は行わない。
    // ===================================================

    Logger::info("RouteFollower: "
                 "QR1 position = QR2 detection point "
                 "-> additional straight skipped");
  }

  // =====================================================
  // QR①をスキップする場合
  // =====================================================

  else {
    Logger::warning("RouteFollower: "
                    "===== QR1 SKIPPED =====");

    /*
     * QR②位置
     *
     *   ゲート中心 +125mm
     *
     * QR②検出位置
     *
     *   +125 -250
     *
     * = ゲート中心 -125mm
     *
     * = QR①位置
     */

    double distanceToSecondDetection
        = distanceToGate + QR_TO_GATE_DISTANCE - SQUARE_DETECTION_DISTANCE;

    if(distanceToSecondDetection < 0.0) {
      distanceToSecondDetection = 0.0;
    }

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "Move directly to QR2 detection point "
                      "%.2f mm",
                      distanceToSecondDetection);

    if(distanceToSecondDetection > 0.0) {
      straight(distanceToSecondDetection);
    }
  }

  // =====================================================
  // QR②検出
  // =====================================================

  Logger::info("RouteFollower: "
               "========== QR2 ==========");

  SquareAngleAdjustment::Result secondResult{};

  const bool secondDetected = detectSquare(secondResult);

  // =====================================================
  // QR②検出成功
  // =====================================================

  if(secondDetected) {
    const double secondAngle = secondResult.correctionAngle;

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR2 correction angle=%.2f deg",
                      secondAngle);

    // QR②方向へ回頭
    rotateForSquare(secondAngle);

    // QR②まで進む
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "Move to QR2 %.2f mm",
                      secondResult.straightDistance);

    straight(secondResult.straightDistance);
  }

  // =====================================================
  // QR②検出失敗
  //
  // 回頭は行わない。
  //
  // QR②の250mm手前にいるため、
  // そのまま250mm通常直進する。
  // =====================================================

  else {
    Logger::warning("RouteFollower: "
                    "QR2 detection failed "
                    "-> no rotation");

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR2 fallback straight %.2f mm",
                      SQUARE_DETECTION_DISTANCE);

    straight(SQUARE_DETECTION_DISTANCE);
  }

  // =====================================================
  // QR②から区間終端まで
  // =====================================================

  const double nominalQr2Position = distanceToGate + QR_TO_GATE_DISTANCE;

  double remainingDistance = distance - nominalQr2Position;

  if(remainingDistance < 0.0) {
    remainingDistance = 0.0;
  }

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "remainingDistance=%.2f mm",
                    remainingDistance);

  if(remainingDistance > 0.0) {
    straight(remainingDistance);
  }

  Logger::info("RouteFollower: "
               "===== GATE SEGMENT FINISHED =====");
}

const Gate* RouteFollower::findGate(const RouteState& from, const RouteState& to) const
{
  for(const Gate& gate : mapData.getGates()) {
    const std::vector<GatePass> passes = mapData.getGatePasses(gate.color);

    for(const GatePass& pass : passes) {
      // =====================================================
      // Y方向
      // =====================================================

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

      // =====================================================
      // X方向
      // =====================================================

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

    return gateY == 0 || gateY == SystemInfo::Y_GRID_NUM;
  }

  // 縦向きゲート
  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;

    return gateX == 0 || gateX == SystemInfo::X_GRID_NUM;
  }

  return false;
}

double RouteFollower::calculateDistanceToGate(const RouteState& from, const Gate& gate) const
{
  const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);

  // =====================================================
  // 横向きゲート
  // =====================================================

  if(gate.start.y == gate.end.y) {
    const int centerX = (gate.start.x + gate.end.x) / 2;

    const int gateY = gate.start.y;

    const EtRallyMap::Node gateNode = map.getNode(centerX, gateY);

    return std::abs(gateNode.y - fromNode.y);
  }

  // =====================================================
  // 縦向きゲート
  // =====================================================

  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;

    const int centerY = (gate.start.y + gate.end.y) / 2;

    const EtRallyMap::Node gateNode = map.getNode(gateX, centerY);

    return std::abs(gateNode.x - fromNode.x);
  }

  Logger::error("RouteFollower: invalid gate");

  return -1.0;
}