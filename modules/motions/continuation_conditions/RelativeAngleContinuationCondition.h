/**
 * @file   RelativeAngleContinuationCondition.h
 * @brief  現在角度を基準に、指定した相対角度に到達したかどうかで動作の継続/終了を判断するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef RELATIVE_ANGLE_CONTINUATION_CONDITION_H
#define RELATIVE_ANGLE_CONTINUATION_CONDITION_H

#include "AngleContinuationCondition.h"

class RelativeAngleContinuationCondition : public AngleContinuationCondition {
 public:
  RelativeAngleContinuationCondition(Robot& robot, double _deltaAngle, double _tolerance = 2.0);
  // 判定開始前の準備処理
  // 現在角度を基準に、相対角度から目標角度を算出する
  void prepare() override;

 private:
  // 回頭したい相対角度
  double deltaAngle;
};

#endif