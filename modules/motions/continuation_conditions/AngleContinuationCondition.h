/**
 * @file   AngleContinuationCondition.h
 * @brief  目標角度との誤差に基づいて動作を継続するかを判定するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef ANGLE_CONTINUATION_CONDITION_H
#define ANGLE_CONTINUATION_CONDITION_H

#include "BaseContinuationCondition.h"
#include "AngleNormalizer.h"
#include <cmath>

class AngleContinuationCondition : public BaseContinuationCondition {
 public:
  /**
   * @brief コンストラクタ
   * @param robot ロボット本体への参照
   * @param _targrtAngle 目標角度(°)
   * @param _tolerance 許容誤差(デフォルトは2.0°)
   * */

  AngleContinuationCondition(Robot& robot, double _targetAngle, double _tolerance = 2.0);

  /**
   *@brief 目標角度との誤差が許容範囲を超えていれば継続、収まっていれば終了
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

  double getTargetAngle() const;

 protected:
  double targetAngle;  // 目標角度(°)

 private:
  double tolerance;  // 許容誤差(°)
};

#endif
