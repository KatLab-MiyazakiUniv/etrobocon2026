/**
 * @file   RelativeRotation.cpp
 * @brief  相対角度で回頭動作を実行するクラス
 * @author okuyama0528　yutaro-1214
 */
#include "RelativeRotation.h"

RelativeRotation::RelativeRotation(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    const Pid::PidGain& _anglePidGain, double _deltaAngle)
  : Rotation(_robot, std::move(_continuationCondition), _anglePidGain), deltaAngle(_deltaAngle)
{
}

void RelativeRotation::prepare()
{
  /**
   * @brief 現在角度の取得と目標角度の算出
   * @details
   * 現在のロボットの角度を取得し、相対角度(deltaAngle)を加算して
   * 正規化した目標角度(targetAngle)を設定する
   */

  double initialAngle = getCurrentAngle();  // 現在の走行体角度を取得する
  targetAngle = normalizeAngle(initialAngle + deltaAngle);
}