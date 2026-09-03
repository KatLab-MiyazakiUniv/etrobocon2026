/**
 * @file   RouteFollower.cpp
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#include "RouteFollower.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "AngleNormalizer.h"
#include "CameraTracking.h"
#include "DistanceCondition.h"
#include "Logger.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "SocketClient.h"
#include "Straight.h"
#include "SystemInfo.h"

namespace {

  /**
   * @brief QRコードを検出するゲート手前距離[mm]
   */
  constexpr double QR_DETECTION_BEFORE_GATE = 125.0;

  /**
   * @brief QR補正を行う走行距離[mm]
   */
  constexpr double QR_TRACKING_DISTANCE = 250.0;

  /**
   * @brief QRコード検出前の停止時間[ms]
   */
  constexpr int QR_DETECTION_STOP_TIME = 500;

  /**
   * @brief 回頭終了判定の許容誤差[deg]
   */
  constexpr double ROTATION_TOLERANCE = 0.2;

}  // namespace

RouteFollower::RouteFollower(
    Robot& _robot,
    const EtRallyMap& _map,
    double _targetSpeed,
    double _qrTrackingSpeed,
    int _qrTargetX,
    const Pid::PidGain& _rotationPid,
    const Pid::PidGain& _rightPid,
    const Pid::PidGain& _leftPid,
    const Pid::PidGain& _straightAnglePid,
    const Pid::PidGain& _qrTrackingPid)
  : robot(_robot),
    map(_map),
    targetSpeed(_targetSpeed),
    qrTrackingSpeed(_qrTrackingSpeed),
    qrTargetX(_qrTargetX),
    rotationPid(_rotationPid),
    rightPid(_rightPid),
    leftPid(_leftPid),
    straightAnglePid(_straightAnglePid),
    qrTrackingPid(_qrTrackingPid)
{
  LOG_CREATE("RouteFollower");
}

void RouteFollower::run(
    const std::vector<RouteState>& route)
{
  if(route.size() < 2) {
    Logger::info(
        "RouteFollower: route is too short");

    return;
  }

  /*
   * QR補正を使用しない通常走行。
   */
  for(std::size_t i = 1;
      i < route.size();
      ++i) {

    const RouteState& from =
        route[i - 1];

    const RouteState& to =
        route[i];

    Logger::printfLog(
        Logger::INFO,
        "RouteFollower[%d]: (%d,%d) -> (%d,%d)",
        static_cast<int>(i),
        from.x,
        from.y,
        to.x,
        to.y);

    // =====================================================
    // 回頭
    // =====================================================

    const double rotationAngle =
        calculateRotationAngle(
            from.direction,
            to.direction);

    if(std::abs(rotationAngle) > 0.001) {
      Logger::printfLog(
          Logger::INFO,
          "Rotate: %.2f deg",
          rotationAngle);

      rotate(rotationAngle);
    }

    // =====================================================
    // 同一地点の場合は方向変更だけで終了
    // =====================================================

    if(from.x == to.x
       && from.y == to.y) {

      continue;
    }

    // =====================================================
    // 直進距離
    // =====================================================

    const double distance =
        calculateDistance(
            from,
            to);

    if(distance <= 0.0) {
      Logger::printfLog(
          Logger::ERROR,
          "Invalid route distance: (%d,%d) -> (%d,%d)",
          from.x,
          from.y,
          to.x,
          to.y);

      robot
          .getWheelMotorControllerInstance()
          .stopBoth();

      return;
    }

    Logger::printfLog(
        Logger::INFO,
        "Straight: %.2f mm",
        distance);

    straight(distance);
  }
}

void RouteFollower::run(
    const std::vector<RouteState>& route,
    const Gate& targetGate)
{
  if(route.size() < 2) {
    Logger::info(
        "RouteFollower: route is too short");

    return;
  }

  /*
   * route[0]は現在位置なので、
   * route[1]から順番に走行する。
   */
  for(std::size_t i = 1;
      i < route.size();
      ++i) {

    const RouteState& from =
        route[i - 1];

    const RouteState& to =
        route[i];

    Logger::printfLog(
        Logger::INFO,
        "RouteFollower[%d]: (%d,%d) -> (%d,%d)",
        static_cast<int>(i),
        from.x,
        from.y,
        to.x,
        to.y);

    // =====================================================
    // 回頭
    // =====================================================

    const double rotationAngle =
        calculateRotationAngle(
            from.direction,
            to.direction);

    if(std::abs(rotationAngle) > 0.001) {
      Logger::printfLog(
          Logger::INFO,
          "Rotate: %.2f deg",
          rotationAngle);

      rotate(rotationAngle);
    }

    // =====================================================
    // 同一地点の場合は方向変更だけで終了
    // =====================================================

    if(from.x == to.x
       && from.y == to.y) {

      continue;
    }

    // =====================================================
    // 直進距離
    // =====================================================

    const double distance =
        calculateDistance(
            from,
            to);

    if(distance <= 0.0) {
      Logger::printfLog(
          Logger::ERROR,
          "Invalid route distance: (%d,%d) -> (%d,%d)",
          from.x,
          from.y,
          to.x,
          to.y);

      robot
          .getWheelMotorControllerInstance()
          .stopBoth();

      return;
    }

    Logger::printfLog(
        Logger::INFO,
        "Straight: %.2f mm",
        distance);

    /*
     * GateRoutePlannerで生成した経路では、
     * 最後の区間に通過対象のゲートが含まれる。
     *
     * 経路圧縮後は、
     *
     * entrance -> exit
     *
     * ではなく、
     *
     * entranceより手前 -> exit
     *
     * のようになることがあるため、
     * GatePassとの完全一致では判定しない。
     *
     * 呼び出し側から渡されたtargetGateを使用する。
     */
    const bool isLastSegment =
        (i == route.size() - 1);

    if(isLastSegment) {
      runGateSegment(
          from,
          to,
          distance,
          targetGate);

    } else {

      straight(distance);
    }
  }
}

double RouteFollower::directionToHeading(
    Direction direction) const
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

double RouteFollower::calculateRotationAngle(
    Direction from,
    Direction to) const
{
  const double currentHeading =
      directionToHeading(from);

  const double targetHeading =
      directionToHeading(to);

  return AngleNormalizer::normalizeAngle(
      currentHeading - targetHeading);
}

double RouteFollower::calculateDistance(
    const RouteState& from,
    const RouteState& to) const
{
  const EtRallyMap::Node fromNode =
      map.getNode(
          from.x,
          from.y);

  const EtRallyMap::Node toNode =
      map.getNode(
          to.x,
          to.y);

  // =====================================================
  // X方向
  // =====================================================

  if(from.y == to.y) {
    return std::abs(
        toNode.x
        - fromNode.x);
  }

  // =====================================================
  // Y方向
  // =====================================================

  if(from.x == to.x) {
    return std::abs(
        toNode.y
        - fromNode.y);
  }

  /*
   * 圧縮済み経路は縦または横への
   * 移動のみを想定する。
   */
  Logger::printfLog(
      Logger::ERROR,
      "Diagonal route: (%d,%d) -> (%d,%d)",
      from.x,
      from.y,
      to.x,
      to.y);

  return 0.0;
}

void RouteFollower::rotate(
    double angle)
{
  auto condition =
      std::make_unique<RelativeAngleCondition>(
          robot,
          angle,
          ROTATION_TOLERANCE);

  RelativeRotation rotation(
      robot,
      std::move(condition),
      rotationPid,
      angle);

  rotation.run();
}

void RouteFollower::straight(
    double distance)
{
  if(distance <= 0.0) {
    return;
  }

  auto condition =
      std::make_unique<DistanceCondition>(
          robot,
          distance);

  Straight straightMotion(
      robot,
      std::move(condition),
      targetSpeed,
      rightPid,
      leftPid,
      straightAnglePid,
      true);

  straightMotion.run();
}

void RouteFollower::straightWithQrCorrection(
    double distance)
{
  if(distance <= 0.0) {
    return;
  }

  auto condition =
      std::make_unique<DistanceCondition>(
          robot,
          distance);

  CameraServer::QrCodeDetectorRequest qrRequest;

  /*
   * QR追従専用設定を使用する。
   *
   * speed:
   *   qrTrackingSpeed
   *
   * targetX:
   *   qrTargetX
   *
   * PID:
   *   qrTrackingPid
   */
  CameraTracking cameraTracking(
      robot,
      std::move(condition),
      qrTrackingSpeed,
      qrTargetX,
      qrTrackingPid,
      qrRequest,
      false);

  cameraTracking.run();
}

bool RouteFollower::detectQrCode()
{
  SocketClient& client =
      robot.getCameraSocketClientInstance();

  CameraServer::QrCodeDetectorRequest request;

  CameraServer::QrCodeDetectorResponse response;

  const bool success =
      client.executeQrCodeDetection(
          request,
          response);

  if(!success) {
    Logger::warning(
        "RouteFollower: QR detection communication failed");

    return false;
  }

  if(!response.wasDetected) {
    Logger::info(
        "RouteFollower: QR code was not detected");

    return false;
  }

  Logger::info(
      "RouteFollower: QR code detected");

  return true;
}

bool RouteFollower::isOuterGate(
    const Gate& gate) const
{
  // =====================================================
  // 横向きゲート
  // =====================================================

  if(gate.start.y
     == gate.end.y) {

    const int gateY =
        gate.start.y;

    /*
     * 上端または下端にあるゲートは外周。
     */
    return gateY == 0
           || gateY
                  == SystemInfo::Y_GRID_NUM;
  }

  // =====================================================
  // 縦向きゲート
  // =====================================================

  if(gate.start.x
     == gate.end.x) {

    const int gateX =
        gate.start.x;

    /*
     * 左端または右端にあるゲートは外周。
     */
    return gateX == 0
           || gateX
                  == SystemInfo::X_GRID_NUM;
  }

  return false;
}

double RouteFollower::calculateDistanceToGate(
    const RouteState& from,
    const Gate& gate) const
{
  const EtRallyMap::Node fromNode =
      map.getNode(
          from.x,
          from.y);

  // =====================================================
  // 横向きゲート
  // =====================================================

  if(gate.start.y
     == gate.end.y) {

    const int centerX =
        (gate.start.x
         + gate.end.x)
        / 2;

    const int gateY =
        gate.start.y;

    const EtRallyMap::Node gateNode =
        map.getNode(
            centerX,
            gateY);

    /*
     * 横向きゲートはY方向に通過するので、
     * Y座標差を距離として使用する。
     */
    return std::abs(
        gateNode.y
        - fromNode.y);
  }

  // =====================================================
  // 縦向きゲート
  // =====================================================

  if(gate.start.x
     == gate.end.x) {

    const int gateX =
        gate.start.x;

    const int centerY =
        (gate.start.y
         + gate.end.y)
        / 2;

    const EtRallyMap::Node gateNode =
        map.getNode(
            gateX,
            centerY);

    /*
     * 縦向きゲートはX方向に通過するので、
     * X座標差を距離として使用する。
     */
    return std::abs(
        gateNode.x
        - fromNode.x);
  }

  Logger::error(
      "RouteFollower: invalid gate");

  return 0.0;
}

void RouteFollower::runGateSegment(
    const RouteState& from,
    const RouteState& to,
    double distance,
    const Gate& gate)
{
  /*
   * 現状toは距離計算済みのため、
   * この関数内では直接使用しない。
   */
  (void)to;

  // =====================================================
  // 1. 外周ゲート判定
  // =====================================================

  if(isOuterGate(gate)) {
    Logger::info(
        "RouteFollower: outer gate -> Straight");

    straight(distance);

    return;
  }

  // =====================================================
  // 2. 区間開始地点からゲートまでの距離
  // =====================================================

  const double distanceToGate =
      calculateDistanceToGate(
          from,
          gate);

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: distance to gate = %.2f mm",
      distanceToGate);

  /*
   * ゲート125mm手前を
   * QRコード検出地点とする。
   */
  const double beforeQrDistance =
      distanceToGate
      - QR_DETECTION_BEFORE_GATE;

  /*
   * 開始地点がすでにQR検出地点を
   * 通過している場合は通常直進する。
   */
  if(beforeQrDistance < 0.0) {
    Logger::warning(
        "RouteFollower: QR detection point is behind start -> Straight");

    straight(distance);

    return;
  }

  /*
   * ゲートが現在の走行区間の外にある場合は、
   * QR補正せず通常直進する。
   */
  if(beforeQrDistance > distance) {
    Logger::warning(
        "RouteFollower: gate is outside segment -> Straight");

    straight(distance);

    return;
  }

  // =====================================================
  // 3. ゲート125mm手前まで通常Straight
  // =====================================================

  if(beforeQrDistance > 0.0) {
    Logger::printfLog(
        Logger::INFO,
        "RouteFollower: Straight before QR = %.2f mm",
        beforeQrDistance);

    straight(
        beforeQrDistance);
  }

  // =====================================================
  // 4. QRコード検出前に一時停止
  // =====================================================

  Logger::info(
      "RouteFollower: stop before QR detection");

  robot
      .getWheelMotorControllerInstance()
      .stopBoth();

  /*
   * Straight終了直後は車体が完全に
   * 静止していない可能性があるため、
   * 500ms待ってからQRコードを撮影する。
   */
  std::this_thread::sleep_for(
      std::chrono::milliseconds(
          QR_DETECTION_STOP_TIME));

  // =====================================================
  // 5. QRコードを1回検出
  // =====================================================

  Logger::info(
      "RouteFollower: QR detection start");

  const bool qrDetected =
      detectQrCode();

  // =====================================================
  // 6. QR検出地点から区間終了までの距離
  // =====================================================

  const double availableDistance =
      distance
      - beforeQrDistance;

  /*
   * QR補正走行は基本250mm。
   *
   * 区間の残り距離が250mm未満の場合は
   * 区間外へ出ないように残り距離までとする。
   */
  const double qrSectionDistance =
      std::min(
          QR_TRACKING_DISTANCE,
          availableDistance);

  // =====================================================
  // 7. QR追従または通常直進
  // =====================================================

  if(qrDetected) {
    Logger::printfLog(
        Logger::INFO,
        "RouteFollower: QR detected -> CameraTracking %.2f mm",
        qrSectionDistance);

    straightWithQrCorrection(
        qrSectionDistance);

  } else {

    Logger::printfLog(
        Logger::INFO,
        "RouteFollower: QR not detected -> Straight %.2f mm",
        qrSectionDistance);

    straight(
        qrSectionDistance);
  }

  // =====================================================
  // 8. 残り距離を通常Straight
  // =====================================================

  const double remainingDistance =
      distance
      - beforeQrDistance
      - qrSectionDistance;

  if(remainingDistance > 0.0) {
    Logger::printfLog(
        Logger::INFO,
        "RouteFollower: remaining Straight = %.2f mm",
        remainingDistance);

    straight(
        remainingDistance);
  }
}