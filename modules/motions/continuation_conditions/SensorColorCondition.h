/**
 * @file   SensorColorCondition.h
 * @brief  目標色を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#ifndef SENSOR_COLOR_CONDITION_H
#define SENSOR_COLOR_CONDITION_H

#include "BaseContinuationCondition.h"
#include "ColorSensorController.h"
#include <cmath>
#include "Logger.h"

class SensorColorCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief SensorColorCondition を初期化する
   * @param _robot robotクラスのインスタンスの参照
   * @param _targetColor 指定色
   */
  SensorColorCondition(Robot& _robot, ColorSensorController::COLOR _targetColor);

  /**
   * デストラクタ
   */
  ~SensorColorCondition();

  /**
   * @brief 指定色を検出する場合、動作を継続しないと判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

 private:
  int colorCount = 0;                        // 指定色を取得した回数
  ColorSensorController::COLOR targetColor;  // 指定色
  static constexpr int JUDGE_COUNT = 2;      // 継続条件を修了するために必要な連続取得回数
};

#endif