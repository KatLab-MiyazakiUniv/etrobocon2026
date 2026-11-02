
/**
 * @file   ColorCalibrator.h
 * @brief  カラーセンサーのHSV基準値を取得・保存するクラス
 */

#ifndef COLOR_CALIBRATOR_H
#define COLOR_CALIBRATOR_H

#include "BaseMotion.h"
#include "RepeatCountCondition.h"
#include "ColorData.h"
#include <vector>

class ColorCalibrator : public BaseMotion {
 public:
  /**
   * @brief ColorCalibratorを初期化する
   * @param _robot Robotクラスのインスタンス
   * @param _targetColor キャリブレーション対象の色
   * @param _sampleCount HSV値の取得回数
   */
  ColorCalibrator(Robot& _robot, ColorSensorController::COLOR _targetColor, int _sampleCount);

 protected:
  /**
   * @brief キャリブレーションを開始できるか判定する
   */
  bool canStart() override;

  /**
   * @brief HSV取得用のデータを初期化する
   */
  void prepare() override;

  /**
   * @brief HSV値を1回取得する
   */
  void executeStep() override;

  /**
   * @brief HSV値の平均を計算してCSVに保存する
   */
  void finish() override;

 private:
  ColorSensorController::COLOR targetColor;
  int sampleCount;
  std::vector<ColorSensorController::HSV> hsvSamples;

  /**
   * @brief HSV値の平均を計算する
   * @return 平均HSV値
   */
  ColorSensorController::HSV calculateAverageHSV() const;
};

#endif
