/**
 * @file   AbsoluteRotation.cpp
 * @brief  絶対角度で回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#include "AbsoluteRotation.h"

AbsoluteRotation::AbsoluteRotation(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    const Pid::PidGain& _anglePidGain, double _targetAbsAngle)
  : Rotation(_robot, std::move(_continuationCondition), _anglePidGain),
    targetAbsAngle(_targetAbsAngle)
{
}

void AbsoluteRotation::prepare()
{
  targetAngle = targetAbsAngle;
}
