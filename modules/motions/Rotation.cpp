/**
 * @file   Rotation.cpp
 * @brief  その場回転動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#include "Rotation.h"

Rotation::Rotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    pid(0.8, 0.0, 0.05, 0.0),
    currentRightPower(0),
    currentLeftPower(0)
{
}

double Rotation::getCurrentAngle()
{
  return robot.getIMUControllerInstance().getAzimuth();
}

void Rotation::executeStep()
{
  double current = getCurrentAngle();
  double target = getTargetAngle();

  double error = target - current;

  while(error > 180) error -= 360;
  while(error < -180) error += 360;

  double turn = pid.calculatePid(error, 0);

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