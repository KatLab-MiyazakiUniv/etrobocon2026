/**
 * @file   AbsoluteAngleCondition.cpp
 * @brief  絶対方位角を基準に動作継続を判定するクラス
 * @author yutaro-1214
 */

#include "AbsoluteAngleCondition.h"

AbsoluteAngleCondition::AbsoluteAngleCondition(Robot& _robot, float _targetAzimuth,
                                               float _angleTolerance)
  : BaseContinuationCondition(_robot),
    targetAzimuth(_targetAzimuth),
    angleTolerance(_angleTolerance)
{
}

void AbsoluteAngleCondition::prepare()
{
  // 初期化不要
}

bool AbsoluteAngleCondition::shouldContinue()
{
  // 現在の方位角を取得
  float currentAzimuth = robot.getIMUControllerInstance().getAzimuth();

  // 目標方位角との差
  float diff = targetAzimuth - currentAzimuth;

  // 許容誤差以内なら終了
  if(std::fabs(diff) <= angleTolerance) return false;

  return true;
}