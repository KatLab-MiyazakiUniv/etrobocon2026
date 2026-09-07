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
#include "SquareAngleAdjustment.h"
#include "Straight.h"

namespace {

  /**
   * @brief 回頭終了許容誤差[deg]
   */
  constexpr double ROTATION_TOLERANCE = 1.0;

  /**
   * @brief 正方形角度補正終了許容誤差[deg]
   *
   * 原因確認のため一旦小さくしている。
   */
  constexpr double SQUARE_ANGLE_TOLERANCE = 0.2;

  /**
   * @brief 1回あたりの最大回頭角度[deg]
   */
  constexpr double RIGHT_ANGLE = 90.0;

  /**
   * @brief 回頭を実行する最低角度[deg]
   */
  constexpr double MIN_ROTATION_ANGLE = 0.001;

  /**
   * @brief 制御変更前の停止時間[ms]
   */
  constexpr int MOTION_CHANGE_SLEEP_TIME = 200;

  /**
   * @brief 制御変更前に停止して待機する
   */
  void waitBeforeMotion(Robot& robot)
  {
    robot.getWheelMotorControllerInstance().stopBoth();

    ClockUtil::sleep(MOTION_CHANGE_SLEEP_TIME);
  }

}  // namespace

RouteFollower::RouteFollower(Robot& _robot, const EtRallyMap& _map, double _targetSpeed,
                             const Pid::PidGain& _rotationPid, const Pid::PidGain& _rightPid,
                             const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid,
                             const Pid::PidGain& _squareAnglePid)
  : robot(_robot),
    map(_map),
    targetSpeed(_targetSpeed),
    rotationPid(_rotationPid),
    rightPid(_rightPid),
    leftPid(_leftPid),
    straightAnglePid(_straightAnglePid),
    squareAnglePid(_squareAnglePid)
{
  LOG_CREATE("RouteFollower");
}

void RouteFollower::run(const std::vector<RouteState>& route)
{
  if(route.size() < 2) {
    Logger::info("RouteFollower: "
                 "route is too short");

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
    // 回頭角度
    // =====================================================

    const double rotationAngle = calculateRotationAngle(from.direction, to.direction);

    if(std::abs(rotationAngle) > MIN_ROTATION_ANGLE) {
      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "rotation required %.2f deg",
                        rotationAngle);

      rotateWithSquareCorrection(rotationAngle);
    }

    // =====================================================
    // 方向変更のみ
    // =====================================================

    if(from.x == to.x && from.y == to.y) {
      continue;
    }

    // =====================================================
    // Straight
    // =====================================================

    const double distance = calculateDistance(from, to);

    if(distance <= 0.0) {
      Logger::error("RouteFollower: "
                    "invalid distance");

      robot.getWheelMotorControllerInstance().stopBoth();

      return;
    }

    straight(distance);
  }

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

  return AngleNormalizer::normalizeAngle(currentHeading - targetHeading);
}

double RouteFollower::calculateDistance(const RouteState& from, const RouteState& to) const
{
  const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);

  const EtRallyMap::Node toNode = map.getNode(to.x, to.y);

  if(from.y == to.y) {
    return std::abs(toNode.x - fromNode.x);
  }

  if(from.x == to.x) {
    return std::abs(toNode.y - fromNode.y);
  }

  return 0.0;
}

void RouteFollower::rotate(double angle)
{
  if(std::abs(angle) <= MIN_ROTATION_ANGLE) {
    return;
  }

  // =====================================================
  // 制御変更前停止
  // =====================================================

  waitBeforeMotion(robot);

  auto condition = std::make_unique<RelativeAngleCondition>(robot, angle, ROTATION_TOLERANCE);

  RelativeRotation rotation(robot, std::move(condition), rotationPid, angle);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Rotation START %.2f",
                    angle);

  rotation.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Rotation FINISHED %.2f",
                    angle);
}

void RouteFollower::rotateWithSquareCorrection(double angle)
{
  double remainingAngle = angle;

  while(std::abs(remainingAngle) > MIN_ROTATION_ANGLE) {
    double stepAngle = remainingAngle;

    // =====================================================
    // 180度等は90度ずつに分割
    // =====================================================

    if(std::abs(remainingAngle) > RIGHT_ANGLE) {
      stepAngle = std::copysign(RIGHT_ANGLE, remainingAngle);
    }

    // =====================================================
    // 回頭
    // =====================================================

    rotate(stepAngle);

    // =====================================================
    // 回頭したら毎回必ずSquare補正
    // =====================================================

    Logger::info("RouteFollower: "
                 "ROTATION FINISHED "
                 "-> CALL SquareAngleAdjustment");

    const bool adjusted = adjustAngleWithSquare();

    if(!adjusted) {
      Logger::warning("RouteFollower: "
                      "SquareAngleAdjustment skipped");
    }

    remainingAngle -= stepAngle;
  }
}

void RouteFollower::straight(double distance)
{
  if(distance <= 0.0) {
    return;
  }

  // =====================================================
  // 制御変更前停止
  // =====================================================

  waitBeforeMotion(robot);

  auto condition = std::make_unique<DistanceCondition>(robot, distance);

  Straight straightMotion(robot, std::move(condition), targetSpeed, rightPid, leftPid,
                          straightAnglePid, true);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Straight START %.2f",
                    distance);

  straightMotion.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Straight FINISHED %.2f",
                    distance);
}

bool RouteFollower::adjustAngleWithSquare()
{
  // =====================================================
  // 制御変更前停止
  // =====================================================

  Logger::info("RouteFollower: "
               "wait before SquareAngleAdjustment");

  waitBeforeMotion(robot);

  // =====================================================
  // ROI
  // =====================================================

  CameraServer::SquareDetectorRequest squareRequest{};

  squareRequest.roi.x = 0;

  squareRequest.roi.y = 0;

  squareRequest.roi.width = CAM_MAX_WIDTH;

  squareRequest.roi.height = CAM_MAX_HEIGHT;

  // =====================================================
  // SquareAngleAdjustment
  // =====================================================

  SquareAngleAdjustment adjustment(robot, squareRequest, squareAnglePid, SQUARE_ANGLE_TOLERANCE);

  Logger::info("RouteFollower: "
               "========== SquareAngleAdjustment START ==========");

  const bool success = adjustment.run();

  if(!success) {
    Logger::warning("RouteFollower: "
                    "========== SquareAngleAdjustment SKIP ==========");

    return false;
  }

  Logger::info("RouteFollower: "
               "========== SquareAngleAdjustment SUCCESS ==========");

  return true;
}