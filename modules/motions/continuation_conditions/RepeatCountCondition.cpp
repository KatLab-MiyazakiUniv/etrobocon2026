/**
 * @file   RepeatCountCondition.cpp
 * @brief  目標繰り返し回数を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#include "RepeatCountCondition.h"

RepeatCountCondition::RepeatCountCondition(Robot& _robot, int _targetRepeats)
  : BaseContinuationCondition(_robot), targetRepeats(_targetRepeats)
{
  LOG_CREATE("RepeatCountCondition");
}

RepeatCountCondition::~RepeatCountCondition()
{
  LOG_DESTROY("RepeatCountCondition");
}

bool RepeatCountCondition::shouldContinue()
{
  if(targetRepeats <= 0) {
    Logger::warning("目標繰り返し回数が無効です。動作を終了します。");
    return false;
  }
  // 繰り返し回数が目標繰り返し回数に到達
  if(currentRepeats >= targetRepeats) return false;

  currentRepeats++;
  return true;
}