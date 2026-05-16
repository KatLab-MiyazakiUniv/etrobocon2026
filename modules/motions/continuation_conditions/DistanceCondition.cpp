/**
 * @file   DistanceCondition.cpp
 * @brief  目標距離を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#include "DistanceCondition.h"

DistanceCondition::DistanceCondition(Robot& _robot, double _targetDistance)
  : BaseContinuationCondition(_robot), targetDistance(_targetDistance), initDistance(0.0)
{
}

void DistanceCondition::prepare()
{
  initDistance = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                           robot.getWheelMotorControllerInstance().getLeftCount());
}

bool DistanceCondition::shouldContinue()
{
  // 走行距離が目標距離に到達
  if(fabs(Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount())
          - initDistance)
     >= targetDistance)
    return false;

  return true;
}