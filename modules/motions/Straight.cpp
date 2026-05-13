/**
 * @file   Straight.h
 * @brief  直進動作を実行するクラス
 * @author migaku2645
 */

#include "Straight.h"

Straight::Straight(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   double _targetSpeed)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    speedCalculator(_robot, _targetSpeed)
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
void Straight::wait()
{
  ClockUtil::sleep();  // 10ミリ秒待機
}
