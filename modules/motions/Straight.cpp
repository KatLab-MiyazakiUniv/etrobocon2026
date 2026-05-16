/**
 * @file   Straight.h
 * @brief  直進動作を実行するクラス
 * @author migaku2645
 */

#include "Straight.h"

Straight::Straight(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   double _targetSpeed, Pid::PidGain _rightPid, Pid::PidGain _leftPid)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    speedCalculator(_robot, _rightPid, _leftPid, _targetSpeed)
{
}

void Straight::executeStep()
{
  double requiredRightPower = speedCalculator.calculateRightMotorPower();
  double requiredLeftPower = speedCalculator.calculateLeftMotorPower();

  // モーターにPower値をセット
  robot.getWheelMotorControllerInstance().setRightPower(requiredRightPower);
  robot.getWheelMotorControllerInstance().setLeftPower(requiredLeftPower);
}

void Straight::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}
