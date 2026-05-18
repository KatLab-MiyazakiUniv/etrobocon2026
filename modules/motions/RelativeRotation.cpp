/**
 * @file   RelativeRotation.cpp
 * @brief  相対角度で回頭動作を実行するクラス
 * @author okuyama0528
 */
#include "RelativeRotation.h"

RelativeRotation::RelativeRotation(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    double _deltaAngle)
  : Rotation(_robot, std::move(_continuationCondition)), deltaAngle(_deltaAngle)
{
}

void RelativeRotation::prepare()
{
  Rotation::prepare();

  targetAngle = normalizeAngle(initialAngle + deltaAngle);
}