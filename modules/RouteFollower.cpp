/**
 * @file   RouteFollower.cpp
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#include "RouteFollower.h"

#include <cmath>
#include <memory>

#include "AngleNormalizer.h"
#include "CameraTracking.h"
#include "DistanceCondition.h"
#include "Logger.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "SocketClient.h"
#include "Straight.h"

RouteFollower::RouteFollower(Robot& _robot, const EtRallyMap& _map, double _targetSpeed,
                             const Pid::PidGain& _rotationPid, const Pid::PidGain& _rightPid,
                             const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid)
  : robot(_robot),
    map(_map),
    targetSpeed(_targetSpeed),
    rotationPid(_rotationPid),
    rightPid(_rightPid),
    leftPid(_leftPid),
    straightAnglePid(_straightAnglePid)
{
  LOG_CREATE("RouteFollower");
}

void RouteFollower::run(const std::vector<RouteState>& route)
{
  run(route, false);
}

void RouteFollower::run(const std::vector<RouteState>& route, bool useQrCorrection)
{
  if(route.size() < 2) {
    Logger::info("RouteFollower: route is too short");
    return;
  }

  /*
   * route[0]は現在位置なのでroute[1]から走行する。
   */
  for(std::size_t i = 1; i < route.size(); ++i) {
    const RouteState& from = route[i - 1];
    const RouteState& to = route[i];

    Logger::printfLog(Logger::INFO, "RouteFollower[%d]: (%d,%d) -> (%d,%d)",
                      static_cast<int>(i), from.x, from.y, to.x, to.y);

    // =====================================================
    // 回頭
    // =====================================================

    const double rotationAngle =
        calculateRotationAngle(from.direction, to.direction);

    if(std::abs(rotationAngle) > 0.001) {
      Logger::printfLog(Logger::INFO, "Rotate: %.2f deg", rotationAngle);

      rotate(rotationAngle);
    }

    // =====================================================
    // 同じグリッドなら方向変更だけで終了
    // =====================================================

    if(from.x == to.x && from.y == to.y) {
      continue;
    }

    // =====================================================
    // Straightで走る距離を計算
    // =====================================================

    const double distance = calculateDistance(from, to);

    if(distance <= 0.0) {
      Logger::printfLog(Logger::ERROR,
                        "Invalid route distance: (%d,%d) -> (%d,%d)",
                        from.x, from.y, to.x, to.y);

      robot.getWheelMotorControllerInstance().stopBoth();

      return;
    }

    Logger::printfLog(Logger::INFO, "Straight: %.2f mm", distance);

    /*
     * QR補正を使用する場合、
     * このRouteの最後の直進区間をQR補正対象とする。
     *
     * GateRoutePlannerでは、
     *   entrance -> exit
     * のゲート通過区間がrouteの最後に追加されるため、
     * 最後の区間をゲート通過区間として扱う。
     */
    const bool isLastSegment = (i == route.size() - 1);

    if(useQrCorrection && isLastSegment) {
      Logger::info("RouteFollower: QR correction check");

      if(detectQrCode()) {
        Logger::info("RouteFollower: QR detected -> CameraTracking");

        straightWithQrCorrection(distance);
      } else {
        Logger::info("RouteFollower: QR not detected -> Straight");

        straight(distance);
      }
    } else {
      straight(distance);
    }
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

double RouteFollower::calculateDistance(const RouteState& from,
                                        const RouteState& to) const
{
  const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);
  const EtRallyMap::Node toNode = map.getNode(to.x, to.y);

  // X方向の移動
  if(from.y == to.y) {
    return std::abs(toNode.x - fromNode.x);
  }

  // Y方向の移動
  if(from.x == to.x) {
    return std::abs(toNode.y - fromNode.y);
  }

  /*
   * 圧縮済み経路は縦か横への移動のみを想定
   */
  Logger::printfLog(Logger::ERROR,
                    "Diagonal route: (%d,%d) -> (%d,%d)",
                    from.x, from.y, to.x, to.y);

  return 0.0;
}

void RouteFollower::rotate(double angle)
{
  constexpr double ROTATION_TOLERANCE = 2.0;

  auto condition =
      std::make_unique<RelativeAngleCondition>(
          robot, angle, ROTATION_TOLERANCE);

  RelativeRotation rotation(
      robot, std::move(condition), rotationPid, angle);

  rotation.run();
}

void RouteFollower::straight(double distance)
{
  auto condition =
      std::make_unique<DistanceCondition>(robot, distance);

  /*
   * 既存Straightをそのまま使用
   */
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

void RouteFollower::straightWithQrCorrection(double distance)
{
  auto condition =
      std::make_unique<DistanceCondition>(robot, distance);

  /*
   * カメラ画像の中央X座標。
   *
   * 現在のカメラ画像が640px幅の場合は320。
   * カメラ画像サイズが違う場合は変更する。
   */
  constexpr int CAMERA_CENTER_X = 320;

  CameraServer::QrCodeDetectorRequest qrRequest;

  CameraTracking cameraTracking(
      robot,
      std::move(condition),
      targetSpeed,
      CAMERA_CENTER_X,
      straightAnglePid,
      qrRequest,
      true);

  cameraTracking.run();
}

bool RouteFollower::detectQrCode()
{
  SocketClient& client =
      robot.getCameraSocketClientInstance();

  CameraServer::QrCodeDetectorRequest request;
  CameraServer::QrCodeDetectorResponse response;

  const bool success =
      client.executeQrCodeDetection(request, response);

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