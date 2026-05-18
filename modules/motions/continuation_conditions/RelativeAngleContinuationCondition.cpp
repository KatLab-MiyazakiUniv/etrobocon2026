#include "RelativeAngleContinuationCondition.h"

RelativeAngleContinuationCondition::RelativeAngleContinuationCondition(Robot& robot,
                                                                       double _deltaAngle,
                                                                       double _tolerance)
  : AngleContinuationCondition(robot, _tolerance), deltaAngle(_deltaAngle)
{
}

void RelativeAngleContinuationCondition::prepare()
{
  double initialAngle = robot.getIMUControllerInstance().getAzimuth();
  targetAngle = initialAngle + deltaAngle;
}
