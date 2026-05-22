/**
 * @file ColorSensorController.cpp
 * @brief カラーセンサを制御するラッパークラス
 * @author sadomiya-sousi
 */

#include "ColorSensorController.h"

ColorSensorController::ColorSensorController() : colorSensor(EPort::PORT_E) {}

COLOR ColorSensorController::convertStringToColor(std::string_view str)
{
  if(str == "BLACK") return COLOR::BLACK;
  if(str == "WHITE") return COLOR::WHITE;
  if(str == "BLUE") return COLOR::BLUE;
  if(str == "GREEN") return COLOR::GREEN;
  if(str == "YELLOW") return COLOR::YELLOW;
  if(str == "RED") return COLOR::RED;
  return COLOR::NONE;
}

const char* ColorSensorController::convertColorToString(const COLOR& color)
{
  switch(color) {
    case COLOR::BLACK:
      return "BLACK";
    case COLOR::WHITE:
      return "WHITE";
    case COLOR::BLUE:
      return "BLUE";
    case COLOR::GREEN:
      return "GREEN";
    case COLOR::YELLOW:
      return "YELLOW";
    case COLOR::RED:
      return "RED";
    default:
      return "NONE";
  }
}

COLOR ColorSensorController::convertHsvToColor(HSV& hsv)
{
  // 明度が極端に低ければ、黒を返す
  if(hsv.v < BLACK_LIMIT_BORDER) return COLOR::BLACK;
  // 明度が極端に高ければ、白を返す
  if(hsv.v > WHITE_LIMIT_BORDER) return COLOR::WHITE;
  // 彩度が低い場合
  if(hsv.s < SATURATION_BORDER) {
    // 明度が低ければ、黒を返す
    if(hsv.v < BLACK_BORDER) return COLOR::BLACK;
    // 明度が高ければ、白を返す
    return COLOR::WHITE;
  }
  // 各色相の境界によって、色を判別する
  if(hsv.h < RED_BORDER) return COLOR::RED;
  if(hsv.h < YELLOW_BORDER) return COLOR::YELLOW;
  if(hsv.h < GREEN_BORDER) return COLOR::GREEN;
  if(hsv.h < BLUE_BORDER) return COLOR::BLUE;
  return COLOR::RED;
}

// 反射光強度を0~100の範囲で取得
int ColorSensorController::getReflectance()
{
  return (colorSensor.getReflection());
}

// 消灯して環境光を取得(0-100)
int ColorSensorController::getAmbient()
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

// ライトの輝度を個別に設定する
void ColorSensorController::setLightBrightness(int left, int center, int right)
{
  colorSensor.setLight(left, center, righ / t);
}