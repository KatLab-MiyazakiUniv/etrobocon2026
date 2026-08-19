/**
 * @file   GoalNavigation.cpp
 * @brief  目標地点への回頭と直進を実行するクラス
 * @author yutaro-1214
 */

#include "GoalNavigation.h"

#include <cmath>
#include <memory>
#include <utility>

GoalNavigation::GoalNavigation(Robot& _robot,
                               std::unique_ptr<BaseContinuationCondition> _ContinuationCondition,
                               double _targetX, double _targetY, double _targetSpeed,
                               const Pid::PidGain& _rotationPid, const Pid::PidGain& _rightPid,
                               const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid)
  : BaseMotion(_robot, std::move(_ContinuationCondition)),
    navigator(_robot.getNavigatorInstance()),
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
  // 現在位置から目標地点までの距離を計算する
  targetDistance = navigator.calculateDistance(targetX, targetY);

    // 現在位置から目標地点までの方位角を計算する
  targetHeading = navigator.calculateHeading(targetX, targetY);

  Logger::printfLog(Logger::INFO,
                    "GoalNavigation: target=(%.2f, %.2f), "
                    "distance=%.2f, heading=%.2f",
                    targetX, targetY, targetDistance, targetHeading);
}

void GoalNavigation::executeStep()
{
  // すでに目標地点付近にいる場合は何もしない
  if(targetDistance <= DISTANCE_TOLERANCE) {
    return;
  }

  
const double currentHeading= robot.getIMUControllerInstance().getAzimuth();

const double angleDifference= AngleNormalizer::normalizeAngle(targetHeading - currentHeading);
  // 目標方向との角度差が許容誤差より大きい場合だけ回頭する。
  if(std::abs(angleDifference) > ROTATION_TOLERANCE) {
    auto rotationCondition
        = std::make_unique<RelativeAngleCondition>(robot,angleDifference , ROTATION_TOLERANCE);

    RelativeRotation rotation(robot, std::move(rotationCondition), rotationPid, angleDifference);

    rotation.run();
  }

  // 回頭後の位置からの距離を計算する
  targetDistance = navigator.calculateDistance(targetX, targetY);

  // 計算した距離だけ直進する。
  auto distanceCondition = std::make_unique<DistanceCondition>(robot, targetDistance);

  Straight straight(robot, std::move(distanceCondition), targetSpeed, rightPid, leftPid,
                    straightAnglePid, true);

  straight.run();
}

void GoalNavigation::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}