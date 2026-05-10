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
  double currentRightPower = speedCalculator.calculateRightMotorPower();
  double currentLeftPower = speedCalculator.calculateLeftMotorPower();

  // モーターにPower値をセット
  robot.getWheelMotorControllerInstance().setRightPower(currentRightPower);
  robot.getWheelMotorControllerInstance().setLeftPower(currentLeftPower);
}
void Straight::wait()
{
  ClockUtil::sleep();  // 10ミリ秒待機
}
