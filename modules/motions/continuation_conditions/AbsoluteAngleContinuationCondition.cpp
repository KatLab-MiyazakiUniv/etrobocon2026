/**
 * @file   AbsoluteAngleContinuationCondition.cpp
 * @brief  絶対角度による継続条件
 * @author okuyama0528 yutaro-1214
 */
#include "AbsoluteAngleContinuationCondition.h"

/**
 * @brief コンストラクタ
 * @param robot           ロボット本体への参照
 * @param _targetAbsAngle 目標となる絶対角度(°)
 * @param _tolerance      角度判定の許容誤差(°)
 */
AbsoluteAngleContinuationCondition::AbsoluteAngleContinuationCondition(Robot& robot,
                                                                       double _targetAbsAngle,
                                                                       double _tolerance)
  : AngleContinuationCondition(robot, _tolerance), targetAbsAngle(_targetAbsAngle)
{
}

/**
 * @brief 継続条件の準備処理を行う
 *
 * 判定に使用する目標角度として、
 * 指定された絶対角度を設定する。
 */
void AbsoluteAngleContinuationCondition::prepare()
{
  targetAngle = targetAbsAngle;
}
