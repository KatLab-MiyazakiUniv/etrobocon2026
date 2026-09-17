/**
 * @file   RelativeAngleCondition.h
 * @brief  現在角度を基準に、指定した相対角度に到達したかどうかで動作を継続すべきかを判定するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef RELATIVE_ANGLE_CONDITION_H
#define RELATIVE_ANGLE_CONDITION_H

#include "AngleCondition.h"
#include "Logger.h"

class RelativeAngleCondition : public AngleCondition {
 public:
  /**
   * @brief 相対角度で回頭完了を判定するコンストラクタ
   * @param robot 制御対象
   * @param _relativeAngle 回頭する相対角度(°)
   * @param _tolerance 許容誤差(°)
   */
  RelativeAngleCondition(Robot& robot, double _relativeAngle, double _tolerance = 2.0);

  /**
   * デストラクタ
   */
  ~RelativeAngleCondition();

  /**
   * @brief 判定開始前の準備処理
   * @details
   * 現在角度を取得し、相対角度(relativeAngle)を加算して
   * 目標角度(targetAngle)を算出する
   */
  void prepare() override;

 private:
  double relativeAngle;  // 回頭したい相対角度
};

#endif
