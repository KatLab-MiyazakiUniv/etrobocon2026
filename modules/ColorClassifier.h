
/**
 * @file ColorClassifier.h
 * @brief HSV基準値を用いて色を判定するクラス
 */

#ifndef COLOR_CLASSIFIER_H
#define COLOR_CLASSIFIER_H

#include "ColorDataManager.h"

class ColorClassifier {
 public:
  /**
   * @brief コンストラクタ
   * @param _dataManager 色データ管理クラス
   */
  explicit ColorClassifier(const ColorDataManager& _dataManager);

  /**
   * @brief HSV値から色を判定する
   * @param hsv 判定対象のHSV値
   * @return 判定した色。一致しない場合はCOLOR::NONE
   */
  ColorSensorController::COLOR classify(const ColorSensorController::HSV& hsv) const;

 private:
  const ColorDataManager& dataManager;

  static constexpr int H_TOLERANCE = 10;
  static constexpr int S_TOLERANCE = 10;
  static constexpr int V_TOLERANCE = 10;

  /**
   * @brief Hの円環を考慮して差分を計算する
   */
  int calculateHueDifference(uint16_t value, uint16_t reference) const;

  /**
   * @brief HSV値が基準値の許容範囲内か判定する
   */
  bool isInRange(const ColorSensorController::HSV& hsv,
                 const ColorSensorController::HSV& reference) const;

  /**
   * @brief HSV値と基準値の差を計算する
   */
  int calculateDifference(const ColorSensorController::HSV& hsv,
                          const ColorSensorController::HSV& reference) const;
};

#endif
