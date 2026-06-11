/**
 * @file   RunningTimeCondition.cpp
 * @brief  目標時間を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#include "RunningTimeCondition.h"

RunningTimeCondition::RunningTimeCondition(Robot& _robot, int _targetTime)
  : BaseContinuationCondition(_robot), targetTime(_targetTime)
{
  LOG_CREATE("RunningTimeCondition");
}

RunningTimeCondition::~RunningTimeCondition()
{
  LOG_DESTROY("RunningTimeCondition");
}

bool RunningTimeCondition::shouldContinue()
{
  if(targetTime <= 0) {
    Logger::warning("目標時間が無効です");
    return false;
  }

  // 経過時間が目標時間に到達
  if(fabs(ClockUtil::now() - robot.getRunningStartTime()) >= targetTime) return false;

  return true;
}