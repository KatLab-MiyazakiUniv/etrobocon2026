/**
 * @file   RelativeAngleCondition.cpp
 * @brief  現在角度を基準に、指定した相対角度に到達したかどうかで動作を継続すべきかを判定するクラス
 * @author okuyama0528 yutaro-1214
 */
#include "RelativeAngleCondition.h"

RelativeAngleCondition::RelativeAngleCondition(
Robot& robot,
double _relativeAngle,
double _tolerance)
: AngleCondition(robot, 0.0, _tolerance),
relativeAngle(_relativeAngle)
{
LOG_CREATE("RelativeAngleCondition");
}

RelativeAngleCondition::~RelativeAngleCondition()
{
  LOG_DESTROY("RelativeAngleCondition");
}

void RelativeAngleCondition::prepare()
{
double initialAngle
= robot.getIMUControllerInstance().getAzimuth();

targetAngle = AngleNormalizer::normalizeAngle(
initialAngle + relativeAngle);
}