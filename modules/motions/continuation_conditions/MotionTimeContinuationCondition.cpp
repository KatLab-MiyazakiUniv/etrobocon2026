/**
 * @file   MotionTimeContinuationCondition.cpp
 * @brief  目標時間を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#include "MotionTimeContinuationCondition.h"

MotionTimeContinuationCondition::MotionTimeContinuationCondition(Robot& _robot, int _targetTime)
  : BaseContinuationCondition(_robot), targetTime(_targetTime), initTime(0.0)
{
  LOG_CREATE("MotionTimeContinuationCondition");
}

MotionTimeContinuationCondition::~MotionTimeContinuationCondition()
{
  LOG_DESTROY("MotionTimeContinuationCondition");
}

void MotionTimeContinuationCondition::prepare()
{
  initTime = ClockUtil::now();
}

bool MotionTimeContinuationCondition::shouldContinue()
{
  // 経過時間が目標時間に到達
  if(fabs(ClockUtil::now() - initTime) >= targetTime) return false;

  return true;
}