/**
 * @file   AbsoluteRotation.cpp
 * @brief  絶対角度で回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#include "AbsoluteRotation.h"

/**
 * @brief コンストラクタ
 * @param _robot                  ロボット本体への参照
 * @param _continuationCondition  動作継続条件
 * @param _anglePidGain           回頭制御用PIDゲイン
 * @param _specifiedTargetAngle   目標となる絶対角度
 */
AbsoluteRotation::AbsoluteRotation(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    const Pid::PidGain& _anglePidGain, double _specifiedTargetAngle)
  : Rotation(_robot, std::move(_continuationCondition), _anglePidGain),
    specifiedTargetAngle(_specifiedTargetAngle)
{
}

/**
 * @brief 回頭動作の準備処理を行う
 *
 * 回頭制御で使用する目標角度として、
 * 指定された絶対角度を設定する。
 */
void AbsoluteRotation::prepare()
{
  targetAngle = specifiedTargetAngle;
}
