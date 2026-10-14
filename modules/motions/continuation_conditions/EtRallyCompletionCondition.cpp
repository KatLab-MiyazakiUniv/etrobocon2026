/**
 * @file   EtRallyCompletionCondition.cpp
 * @brief  ETラリー全体の完了状態に基づいて動作の継続を判定するクラス
 */

#include "EtRallyCompletionCondition.h"
#include <utility>

EtRallyCompletionCondition::EtRallyCompletionCondition(
    Robot& _robot, std::function<bool()> _notCompleted)
  : BaseContinuationCondition(_robot), notCompleted(std::move(_notCompleted))
{
}

bool EtRallyCompletionCondition::shouldContinue()
{
  return notCompleted();
}