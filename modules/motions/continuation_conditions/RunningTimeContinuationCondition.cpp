/**
 * @file   RunningTimeContinuationCondition.cpp
 * @brief  目標時間を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#include "RunningTimeContinuationCondition.h"

RunningTimeContinuationCondition::RunningTimeContinuationCondition(Robot& _robot, int _targetTime)
  : BaseContinuationCondition(_robot), targetTime(_targetTime)
{
  LOG_CREATE("RunningTimeContinuationCondition");
}

RunningTimeContinuationCondition::~RunningTimeContinuationCondition()
{
  LOG_DESTROY("RunningTimeContinuationCondition");
}

bool RunningTimeContinuationCondition::shouldContinue()
{
  // 経過時間が目標時間に到達
  if(fabs(ClockUtil::now() - robot.getStartTime()) >= targetTime) return false;

  return true;
}