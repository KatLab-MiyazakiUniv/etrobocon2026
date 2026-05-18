/**
 * @file   Rotation.cpp
 * @brief  その場回転動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "Rotation.h"
#include <cmath>
#include <algorithm>

Rotation::Rotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   const Pid::PidGain& _anglePidGain)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    anglePid(_anglePidGain.kp, _anglePidGain.ki, _anglePidGain.kd, 0.0),
    currentRightPower(0),
    currentLeftPower(0),
    targetAngle(0.0)
{
}

double Rotation::getCurrentAngle()
{
  return robot.getIMUControllerInstance().getAzimuth();
}

void Rotation::executeStep()
{
  double currentAngle = getCurrentAngle();

  double error = normalizeAngle(targetAngle - currentAngle);

  double turn = anglePid.calculatePid(error, 0.0);

  turn = std::clamp(turn, -60.0, 60.0);

  currentRightPower = -turn;
  currentLeftPower = turn;

  robot.getWheelMotorControllerInstance().setRightPower(currentRightPower);
  robot.getWheelMotorControllerInstance().setLeftPower(currentLeftPower);
}

void Rotation::finish()
{
  robot.getWheelMotorControllerInstance().setRightPower(0);
  robot.getWheelMotorControllerInstance().setLeftPower(0);
}

double Rotation::normalizeAngle(double angle)
{
  while(angle > 180.0) angle -= 360.0;
  while(angle < -180.0) angle += 360.0;
  return angle;
}
