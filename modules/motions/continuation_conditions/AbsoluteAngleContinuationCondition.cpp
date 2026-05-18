#include "AbsoluteAngleContinuationCondition.h"

AbsoluteAngleContinuationCondition::AbsoluteAngleContinuationCondition(Robot& robot,
                                                                       double _specifiedTargetAngle,
                                                                       double _tolerance)
  : AngleContinuationCondition(robot, _tolerance), specifiedTargetAngle(_specifiedTargetAngle)
{
}

void AbsoluteAngleContinuationCondition::prepare()
{
  targetAngle = specifiedTargetAngle;
}
