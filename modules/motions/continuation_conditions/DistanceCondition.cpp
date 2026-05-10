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