/**
 * @file   RelativeRotation.cpp
 * @brief  相対角度で回頭動作を実行するクラス
 * @author okuyama0528　yutaro-1214
 */
#include "RelativeRotation.h"

RelativeRotation::RelativeRotation(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    const Pid::PidGain& _anglePidGain, double _deltaAngle)
  : Rotation(_robot, std::move(_continuationCondition), _anglePidGain), deltaAngle(_deltaAngle)
{
}

void RelativeRotation::prepare()
{
  // 現在のロボットの角度を取得する
  double initialAngle = getCurrentAngle();
  // 現在角度に回頭したい相対角度（deltaAngle)を足して目標角度を決定する
  targetAngle = normalizeAngle(initialAngle + deltaAngle);
}