/**
 * @file   AngleContinuationCondition.h
 * @brief  目標角度との誤差に基づいて動作の継続/終了を判定するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef ANGLE_CONTINUATION_CONDITION_H
#define ANGLE_CONTINUATION_CONDITION_H

#include "BaseContinuationCondition.h"

class AngleContinuationCondition : public BaseContinuationCondition {
 public:
  AngleContinuationCondition(Robot& robot, double _tolerance = 2.0);
  // 動作を継続するかどうかを判定する関数
  // 目標角度との誤差が許容範囲を超えていれば継続、収まっていれば終了
  bool shouldContinue() override;

 protected:
  // 目標角度
  double targetAngle;
  // 角度を-180〜180の範囲に正規化する関数
  double normalizeAngle(double angle);

 private:
  // 許容誤差（この範囲内なら到達とみなす）
  double tolerance;
};

#endif