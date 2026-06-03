/**
 * @file   AbsoluteAngleContinuationCondition.h
 * @brief  絶対角度による継続条件クラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef ABSOLUTE_ANGLE_CONTINUATION_CONDITION_H
#define ABSOLUTE_ANGLE_CONTINUATION_CONDITION_H

#include "AngleContinuationCondition.h"
#include "Logger.h"

/**
 * @class AbsoluteAngleContinuationCondition
 * @brief 絶対角度に到達したかを判定する継続条件クラス
 * 指定された絶対角度と現在角度を比較し、
 * 許容誤差内に入った場合に条件成立と判定する。
 */
class AbsoluteAngleContinuationCondition : public AngleContinuationCondition {
 public:
  /**
   * @brief コンストラクタ
   * @param robot           ロボット本体への参照
   * @param _targetAbsAngle 目標となる絶対角度(°)
   * @param _tolerance      角度判定の許容誤差(°)
   */
  AbsoluteAngleContinuationCondition(Robot& robot, double _targetAbsAngle, double _tolerance = 2.0);

  /**
   * デストラクタ
   */
  ~AbsoluteAngleContinuationCondition();

  /**
   * @brief 指定された絶対角度を目標角度としてセットする
   */
  void prepare() override;

 private:
  double targetAbsAngle;  // 目標となる絶対角度(°)
};

#endif
