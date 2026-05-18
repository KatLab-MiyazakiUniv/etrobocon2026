/**
 * @file ColorSensorController.cpp
 * @brief カラーセンサを制御するラッパークラス
 * @author sadomiya-sousi
 */

#include "ColorSensorController.h"
#include <iostream>
#include <stdint.h>

ColorSensorController::ColorSensorController(EPort port) : colorSensor(port)
{
  if(colorSensor.hasError()) {
    std::cerr << "ColorSensorController: インスタンス化失敗。ポート: " << static_cast<int>(port)
              << std::endl;
  }
}

// 反射光強度を0~100の範囲で取得
int32_t ColorSensorController::getReflectance()
{
  return (colorSensor.getReflection());
}

// 消灯して環境光を取得(0-100)
int32_t ColorSensorController::getAmbient()
{
  return (colorSensor.getAmbient());
}

// RGB値を取得する(近似なし)
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

// HSV値を取得(特定色への近似あり)
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

// ライトを消灯する(動作しない)
void ColorSensorController::lightOff()
{
  colorSensor.lightOff();
}

// ライトの色を個別に設定する
void ColorSensorController::setLightColor(int r, int g, int b)
{
  colorSensor.setLight(r, g, b);
}

// インスタンス生成が正常ならば0、異常ならば1を返す
bool ColorSensorController::hasError()
{
  return colorSensor.hasError();
}
