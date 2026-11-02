/**
 * @file ColorData.h
 * @brief 色ごとのHSV基準値を保持するデータ構造
 */

#ifndef COLOR_DATA_H
#define COLOR_DATA_H

#include "ColorSensorController.h"

struct ColorData {
  ColorSensorController::COLOR color;
  ColorSensorController::HSV hsv;
  bool acquired;
};

#endif