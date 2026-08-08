/**
 * @file   GoalNavigation.cpp
 * @brief  目標地点への回頭と直進を実行するクラス
 * @author yutaro-1214
 */

#include "GoalNavigation.h"

#include <cmath>
#include <memory>
#include <utility>

GoalNavigation::GoalNavigation(
    Robot& _robot,
    std::unique_ptr<BaseContinuationCondition> _condition,
    double _targetX,
    double _targetY,
    double _targetSpeed,
    const Pid::PidGain& _rotationPid,
    const Pid::PidGain& _rightPid,
    const Pid::PidGain& _leftPid,
    const Pid::PidGain& _straightAnglePid)
    : BaseMotion(
          _robot,
          std::move(_condition)),
      navigator(
          _robot.getNavigatorInstance()),
      targetX(_targetX),
      targetY(_targetY),
      targetSpeed(_targetSpeed),
      targetDistance(0.0),
      targetHeading(0.0),
      rotationPid(_rotationPid),
      rightPid(_rightPid),
      leftPid(_leftPid),
      straightAnglePid(_straightAnglePid)
{
  LOG_CREATE("GoalNavigation");
}

GoalNavigation::~GoalNavigation()
{
  LOG_DESTROY("GoalNavigation");
}

void GoalNavigation::prepare()
{
  /*
   * 現在位置から目標地点までの距離を計算
   */
  targetDistance =
      navigator.calculateDistance(
          targetX,
          targetY);

  /*
   * 現在位置から目標地点への
   * 方位角を計算
   */
  targetHeading =
      navigator.calculateHeading(
          targetX,
          targetY);

  Logger::printfLog(
      Logger::INFO,
      "GoalNavigation: "
      "target=(%.2f, %.2f), "
      "distance=%.2f, "
      "heading=%.2f",
      targetX,
      targetY,
      targetDistance,
      targetHeading);
}

void GoalNavigation::executeStep()
{
  /*
   * =========================
   * 距離を再計算
   * =========================
   */
  targetDistance =
      navigator.calculateDistance(
          targetX,
          targetY);

  /*
   * すでに目標地点付近なら終了
   */
  if(targetDistance <= DISTANCE_TOLERANCE) {
    return;
  }

  /*
   * =========================
   * 目標方向を再計算
   * =========================
   */
  targetHeading =
      navigator.calculateHeading(
          targetX,
          targetY);

  /*
   * =========================
   * 現在の向きを取得
   * =========================
   */
  const double currentHeading =
      robot
          .getIMUControllerInstance()
          .getAzimuth();

  /*
   * =========================
   * 角度差を計算
   * =========================
   */
  const double angleDifference =
      AngleNormalizer::normalizeAngle(
          targetHeading - currentHeading);

  Logger::printfLog(
      Logger::INFO,
      "GoalNavigation: "
      "currentHeading=%.2f, "
      "targetHeading=%.2f, "
      "angleDifference=%.2f, "
      "distance=%.2f",
      currentHeading,
      targetHeading,
      angleDifference,
      targetDistance);

  /*
   * =========================
   * 必要なら回頭
   * =========================
   */
  if(std::abs(angleDifference)
     > ROTATION_TOLERANCE) {

    auto rotationCondition =
        std::make_unique<
            AbsoluteAngleCondition>(
            robot,
            targetHeading,
            ROTATION_TOLERANCE);

    AbsoluteRotation rotation(
        robot,
        std::move(rotationCondition),
        rotationPid,
        targetHeading);

    rotation.run();
  }

  /*
   * =========================
   * 回頭後に距離を再計算
   * =========================
   */
  targetDistance =
      navigator.calculateDistance(
          targetX,
          targetY);

  /*
   * 目標地点付近なら直進しない
   */
  if(targetDistance <= DISTANCE_TOLERANCE) {
    return;
  }

  /*
   * =========================
   * 目標地点まで直進
   * =========================
   */
  auto distanceCondition =
      std::make_unique<DistanceCondition>(
          robot,
          targetDistance);

  Straight straight(
      robot,
      std::move(distanceCondition),
      targetSpeed,
      rightPid,
      leftPid,
      straightAnglePid,
      true);

  straight.run();
}

void GoalNavigation::finish()
{
  robot
      .getWheelMotorControllerInstance()
      .stopBoth();
}