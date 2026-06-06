/**
 * @file   CompoundContinuationCondition.cpp
 * @brief  複数の継続条件を組み合わせて動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#include "CompoundContinuationCondition.h"

CompoundContinuationCondition::CompoundContinuationCondition(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _firstContinuationCondition,
    std::unique_ptr<BaseContinuationCondition> _secondContinuationCondition,
    LogicalOperator _operator)
  : BaseContinuationCondition(_robot),
    firstContinuationCondition(std::move(_firstContinuationCondition)),
    secondContinuationCondition(std::move(_secondContinuationCondition)),
    logicalOperator(_operator)
{
  LOG_CREATE("CompoundContinuationCondition");
}

CompoundContinuationCondition::~CompoundContinuationCondition()
{
  LOG_DESTROY("CompoundContinuationCondition");
}

void CompoundContinuationCondition::prepare()
{
  firstContinuationCondition->prepare();
  secondContinuationCondition->prepare();
}

bool CompoundContinuationCondition::shouldContinue()
{
  if(logicalOperator == LogicalOperator::AND) {
    return firstContinuationCondition->shouldContinue()
           && secondContinuationCondition->shouldContinue();
  } else if(logicalOperator == LogicalOperator::OR) {
    return firstContinuationCondition->shouldContinue()
           || secondContinuationCondition->shouldContinue();
  }
  return false;
}