/**
 * @file   RelativeRotation.cpp
 * @brief  相対角度で回頭動作を実行するクラス
 * @author okuyama0528　yutaro-1214
 */
#include "RelativeRotation.h"

RelativeRotation::RelativeRotation(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    const Pid::PidGain& _anglePidGain, double _relativeAngle)
  : Rotation(_robot, std::move(_continuationCondition), _anglePidGain),
    relativeTargetAngle(_relativeAngle)
{
  LOG_CREATE("RelativeRotation");
}

RelativeRotation::~RelativeRotation()
{
  LOG_DESTROY("RelativeRotation");
}

void RelativeRotation::prepare()
{
  double initialAngle = getCurrentAngle();  // 現在の走行体角度を取得する

  // 現在角度にrelativeAngleを加算し、-180～180度に正規化して目標角度を算出する
  targetAngle = AngleNormalizer::normalizeAngle(initialAngle + relativeTargetAngle);
}