/**
 * @file   AbsoluteAngleCondition.h
 * @brief  絶対方位角を基準に動作継続を判定するクラス
 * @author yutaro-1214
 */

#ifndef ABSOLUTE_ANGLE_CONDITION_H
#define ABSOLUTE_ANGLE_CONDITION_H

#include "BaseContinuationCondition.h"
#include <cmath>

class AbsoluteAngleCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief AbsoluteAngleCondition を初期化する
   * @param _robot Robotクラスのインスタンスの参照
   * @param _targetAzimuth 目標方位角[deg]
   * @param _angleTolerance 許容誤差[deg]
   */
  AbsoluteAngleCondition(Robot& _robot, float _targetAzimuth, float _angleTolerance = 1.0f);

  /**
   * @brief 動作開始前処理
   */
  void prepare() override;

  /**
   * @brief 現在方位角が目標方位角に到達したか判定する
   * @return true  動作継続
   * @return false 動作終了
   */
  bool shouldContinue() override;

 private:
  float targetAzimuth;   // 目標方位角[deg]
  float angleTolerance;  // 許容誤差[deg]
};

#endif