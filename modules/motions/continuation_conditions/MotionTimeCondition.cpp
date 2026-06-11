/**
 * @file   MotionTimeCondition.cpp
 * @brief  目標時間を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#include "MotionTimeCondition.h"

MotionTimeCondition::MotionTimeCondition(Robot& _robot, int _targetTime)
  : BaseContinuationCondition(_robot), targetTime(_targetTime), initTime(0.0)
{
  LOG_CREATE("MotionTimeCondition");
}

MotionTimeCondition::~MotionTimeCondition()
{
  LOG_DESTROY("MotionTimeCondition");
}

void MotionTimeCondition::prepare()
{
  initTime = ClockUtil::now();
}

bool MotionTimeCondition::shouldContinue()
{
  if(targetTime <= 0) {
    Logger::warning("目標時間が無効です");
    return false;
  }

  // 経過時間が目標時間に到達
  if(fabs(ClockUtil::now() - initTime) >= targetTime) return false;

  return true;
}