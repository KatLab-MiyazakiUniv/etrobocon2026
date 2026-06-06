/**
 * @file   ColorContinuationCondition.h
 * @brief  目標色を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#ifndef COLOR_CONTINUATION_CONDITION_H
#define COLOR_CONTINUATION_CONDITION_H

#include "BaseContinuationCondition.h"
#include "ColorSensorController.h"
#include <cmath>
#include "Logger.h"

class ColorContinuationCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief ColorContinuationCondition を初期化する
   * @param _robot robotクラスのインスタンスの参照
   * @param _targetColor 目標色
   */
  ColorContinuationCondition(Robot& _robot, ColorSensorController::COLOR _targetColor);

  /**
   * デストラクタ
   */
  ~ColorContinuationCondition();

  /**
   * @brief 開始時の累計走行距離を取得して initDistance に保存する
   */
  void prepare() override;

  /**
   * @brief 開始時からの移動距離が目標距離に達した場合、動作を継続しないと判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

 private:
  int colorCount = 0;
  ColorSensorController::COLOR targetColor;  // 目標色
  static constexpr int JUDGE_COUNT = 2;      // 取得色の決定に必要な連続取得回数
};

#endif