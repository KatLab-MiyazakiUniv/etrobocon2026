
/**
 * @file ColorClassifier.cpp
 * @brief HSV基準値を用いて色を判定するクラス
 */

#include "ColorClassifier.h"

#include <algorithm>
#include <cmath>

// コンストラクタ
ColorClassifier::ColorClassifier(const ColorDataManager& _dataManager) : dataManager(_dataManager)
{
}

// Hの円環を考慮して差分を計算する
int ColorClassifier::calculateHueDifference(uint16_t value, uint16_t reference) const
{
  const int difference = std::abs(static_cast<int>(value) - static_cast<int>(reference));

  return std::min(difference, 360 - difference);
}

// HSV値が基準値の許容範囲内か判定する
bool ColorClassifier::isInRange(const ColorSensorController::HSV& hsv,
                                const ColorSensorController::HSV& reference) const
{
  const int hDifference = calculateHueDifference(hsv.h, reference.h);

  const int sDifference = std::abs(static_cast<int>(hsv.s) - static_cast<int>(reference.s));

  const int vDifference = std::abs(static_cast<int>(hsv.v) - static_cast<int>(reference.v));

  return hDifference <= H_TOLERANCE && sDifference <= S_TOLERANCE && vDifference <= V_TOLERANCE;
}

// HSV値と基準値の差を計算する
int ColorClassifier::calculateDifference(const ColorSensorController::HSV& hsv,
                                         const ColorSensorController::HSV& reference) const
{
  const int hDifference = calculateHueDifference(hsv.h, reference.h);

  const int sDifference = std::abs(static_cast<int>(hsv.s) - static_cast<int>(reference.s));

  const int vDifference = std::abs(static_cast<int>(hsv.v) - static_cast<int>(reference.v));

  // 各成分の許容幅で正規化した差の合計
  return (hDifference * 100 / H_TOLERANCE) + (sDifference * 100 / S_TOLERANCE)
         + (vDifference * 100 / V_TOLERANCE);
}

// HSV値から色を判定する
ColorSensorController::COLOR ColorClassifier::classify(const ColorSensorController::HSV& hsv) const
{
  using COLOR = ColorSensorController::COLOR;

  // Hの値が想定範囲外の場合は判定しない
  if(hsv.h > 360) {
    return COLOR::NONE;
  }

  const COLOR colors[] = {
    COLOR::BLACK, COLOR::WHITE, COLOR::BLUE, COLOR::GREEN, COLOR::YELLOW, COLOR::RED, COLOR::GRAY,
  };

  COLOR bestColor = COLOR::NONE;
  int minDifference = 1000000;

  for(const COLOR color : colors) {
    const ColorData* data = dataManager.getColorData(color);

    // データが存在しない、または未取得の場合は除外
    if(data == nullptr || !data->acquired) {
      continue;
    }

    if(!isInRange(hsv, data->hsv)) {
      continue;
    }

    const int difference = calculateDifference(hsv, data->hsv);

    if(difference < minDifference) {
      minDifference = difference;
      bestColor = color;
    }
  }

  return bestColor;
}
