/**
 * @file   BaseContinuationCondition.cpp
 * @brief  動作を継続するかを判定するクラスの基底クラス
 * @author takuchi17
 */

#include "BaseContinuationCondition.h"

BaseContinuationCondition::BaseContinuationCondition(Robot& _robot) : robot(_robot) {}

void BaseContinuationCondition::prepare() {}

void BaseContinuationCondition::update() {}