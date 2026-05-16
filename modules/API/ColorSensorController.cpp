/**
 * @file ColorSensorController.cpp
 * @brief カラーセンサを制御するラッパークラス
 * @author sadomiya-sousi
 */

#include "ColorSensorController.h"

using namespace spikeapi;

ColorSensorController::ColorSensorController(EPort port) : colorSensor(port)
{
}

// 反射光強度を取得する
int ColorSensorController::getReflection()
{
  return static_cast<int>(colorSensor.getReflection());
}

// 周囲の明るさを取得する
int ColorSensorController::getAmbient()
{
  return static_cast<int>(colorSensor.getAmbient());
}

// 生のRGB値を取得する
void ColorSensorController::getRawRGB(ColorSensor::RGB& rgb)
{
  colorSensor.getRGB(rgb);
}

// HSV値を取得する (近似なし)
void ColorSensorController::getRawHSV(ColorSensor::HSV& hsv)
{
  colorSensor.getHSV(hsv);
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
