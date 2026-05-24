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
   * @param _tolerance 許容誤差（デフォルト: 2.0度）
   * @details ロボットと許容誤差を設定し、角度判定の初期化を行う
   */

  AngleContinuationCondition(Robot& robot, double _targetAngle, double _tolerance = 2.0);

  bool shouldContinue()
      override;  // 目標角度との誤差が許容範囲を超えていれば継続、収まっていれば終了

  double getTargetAngle() const;

 protected:
  double targetAngle;  // 目標角度

 private:
  double tolerance;  // 許容誤差（この範囲内なら到達とみなす）
};

#endif
