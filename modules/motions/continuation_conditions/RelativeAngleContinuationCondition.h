/**
 * @file   RelativeAngleContinuationCondition.h
 * @brief  現在角度を基準に、指定した相対角度に到達したかどうかで動作を継続すべきかを判定するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef RELATIVE_ANGLE_CONTINUATION_CONDITION_H
#define RELATIVE_ANGLE_CONTINUATION_CONDITION_H

#include "AngleContinuationCondition.h"

class RelativeAngleContinuationCondition : public AngleContinuationCondition {
 public:
  RelativeAngleContinuationCondition(Robot& robot, double _deltaAngle, double _tolerance = 2.0);
  /**
   * @brief 判定開始前の準備処理
   * @details
   * 現在角度を取得し、相対角度(deltaAngle)を加算して
   * 目標角度(targetAngle)を算出する
   */
  void prepare() override;

 private:
  double deltaAngle;  // 回頭したい相対角度
};

#endif
