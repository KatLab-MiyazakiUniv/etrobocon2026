/**
 * @file   AbsoluteAngleContinuationCondition.cpp
 * @brief  絶対角度による継続条件
 * @author okuyama0528 yutaro-1214
 */
#include "AbsoluteAngleContinuationCondition.h"

AbsoluteAngleContinuationCondition::AbsoluteAngleContinuationCondition(Robot& robot,
                                                                       double _targetAbsAngle,
                                                                       double _tolerance)
  : AngleContinuationCondition(robot, _tolerance), targetAbsAngle(_targetAbsAngle)
{
}

void AbsoluteAngleContinuationCondition::prepare()
{
  targetAngle = targetAbsAngle;
}
