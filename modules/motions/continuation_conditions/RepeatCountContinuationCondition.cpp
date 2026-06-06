/**
 * @file   RepeatCountContinuationCondition.cpp
 * @brief  目標繰り返し回数を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#include "RepeatCountContinuationCondition.h"

RepeatCountContinuationCondition::RepeatCountContinuationCondition(Robot& _robot,
                                                                   int _targetRepeats)
  : BaseContinuationCondition(_robot), targetRepeats(_targetRepeats)
{
  LOG_CREATE("RepeatCountContinuationCondition");
}

RepeatCountContinuationCondition::~RepeatCountContinuationCondition()
{
  LOG_DESTROY("RepeatCountContinuationCondition");
}

bool RepeatCountContinuationCondition::shouldContinue()
{
  currentRepeats++;
  // 繰り返し回数が目標繰り返し回数に到達
  if(currentRepeats == targetRepeats) return false;

  return true;
}