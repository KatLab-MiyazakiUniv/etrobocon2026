/**
 * @file ColorSensorController.cpp
 * @brief カラーセンサを制御するラッパークラス
 * @author sadomiya-sousi
 */

#include "ColorSensorController.h"
#include <iostream>  // エラー出力用
#include <stdint.h>  // エラー出力用

ColorSensorController::ColorSensorController(EPort port) : colorSensor(port)
{
  if(colorSensor.hasError()) {
    std::cerr << "ColorSensorController: Failed to initialize color sensor on port "
              << static_cast<int>(port) << std::endl;
  }
}

// 反射光強度を取得する
int32_t ColorSensorController::getReflection()
{
  return (colorSensor.getReflection());
}

// 周囲の明るさを取得する
int32_t ColorSensorController::getAmbient()
{
  return (colorSensor.getAmbient());
}

// 生のRGB値を取得する
void ColorSensorController::getRawRGB(RGB& rgb)
{
  spikeapi::ColorSensor::RGB rawRgb;
  colorSensor.getRGB(rawRgb);
  rgb.r = rawRgb.r;
  rgb.g = rawRgb.g;
  rgb.b = rawRgb.b;
}

// HSV値を取得する (近似なし)
void ColorSensorController::getRawHSV(HSV& hsv, bool surface)
{
  spikeapi::ColorSensor::HSV rawHsv;
  colorSensor.getHSV(rawHsv, surface);
  hsv.h = rawHsv.h;
  hsv.s = rawHsv.s;
  hsv.v = rawHsv.v;
}

// カラーセンサで色を測定する
void ColorSensorController::getColor(HSV& hsv, bool surface)
{
  spikeapi::ColorSensor::HSV rawHsv;
  colorSensor.getColor(rawHsv, surface);
  hsv.h = rawHsv.h;
  hsv.s = rawHsv.s;
  hsv.v = rawHsv.v;
}

// ライトを点灯する
void ColorSensorController::lightOn()
{
  colorSensor.lightOn();
}

// ライトを消灯する
void ColorSensorController::lightOff()
{
  colorSensor.lightOff();
}

// ライトの色を個別に設定する
void ColorSensorController::setLightColor(int r, int g, int b)
{
  colorSensor.setLight(r, g, b);
}

// インスタンス生成が正常にできたかどうかを確認する
bool ColorSensorController::hasError()
{
  return colorSensor.hasError();
}
