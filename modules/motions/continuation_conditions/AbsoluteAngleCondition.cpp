/**
 * @file   AbsoluteAngleCondition.cpp
 * @brief  絶対角度による継続条件
 * @author okuyama0528 yutaro-1214
 */
#include "AbsoluteAngleCondition.h"

AbsoluteAngleCondition::AbsoluteAngleCondition(Robot& robot, double _targetAbsAngle,
                                               double _tolerance)
  : AngleCondition(robot, _tolerance), targetAbsAngle(_targetAbsAngle)
{
  LOG_CREATE("AbsoluteAngleCondition");
}

AbsoluteAngleCondition::~AbsoluteAngleCondition()
{
  LOG_DESTROY("AbsoluteAngleCondition");
}

void AbsoluteAngleCondition::prepare()
{
  targetAngle = targetAbsAngle;
}
