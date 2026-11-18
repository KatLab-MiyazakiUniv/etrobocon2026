/**
 * @file ColorSensorController.cpp
 * @brief カラーセンサを制御するラッパークラス
 * @author sadomiya-sousi
 */

#include "ColorSensorController.h"

ColorSensorController::ColorSensorController() : colorSensor(EPort::PORT_E) {}

ColorSensorController::COLOR ColorSensorController::convertStringToColor(std::string_view str)
{
  if(str == "GRAY") return COLOR::GRAY;
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
    case COLOR::GRAY:
      return "GRAY";
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



ColorSensorController::COLOR ColorSensorController::convertHsvToColor(HSV& hsv)
{
  COLOR detectedColor = COLOR::NONE;

  // 明度が極端に低ければ、黒を返す
  if(hsv.v < BLACK_LIMIT_BORDER) {
    detectedColor = COLOR::BLACK;
  }
  // 明度が極端に高ければ、白を返す
  else if(hsv.v > WHITE_LIMIT_BORDER) {
    detectedColor = COLOR::WHITE;
  }
  // 彩度が低い場合
  else if(hsv.s < SATURATION_BORDER) {
    // 明度が低ければ、黒を返す
    if(hsv.v < BLACK_BORDER) {
      detectedColor = COLOR::BLACK;
    }
    // 黒より明るく白より暗い無彩色は、灰色として区別する。
    else if(hsv.v < GRAY_VALUE_BORDER) {
      detectedColor = COLOR::GRAY;
    }
    // 明度が高ければ、白を返す
    else {
      detectedColor = COLOR::WHITE;
    }
  }
  // この判定保留はSATURATION_BORDER以上の暗色だけに適用する。
  else if(hsv.v < CHROMATIC_VALUE_BORDER) {
    detectedColor = COLOR::NONE;
  }
  // 各色相の境界によって、色を判別する
  else if(hsv.h < RED_BORDER) {
    detectedColor = COLOR::RED;
  }
  else if(hsv.h < YELLOW_BORDER) {
    detectedColor = COLOR::YELLOW;
  }
  else if(hsv.h < GREEN_BORDER) {
    detectedColor = COLOR::GREEN;
  }
  else if(hsv.h < BLUE_BORDER) {
    detectedColor = COLOR::BLUE;
  }
  else {
    detectedColor = COLOR::RED;
  }

  // 測定されたHSVと、判定された色文字列を一緒にログ出力
  Logger::printfLog(Logger::DEBUG, "ColorSensor: h[%u] s[%u] v[%u] -> %s", 
                    hsv.h, hsv.s, hsv.v, convertColorToString(detectedColor));

  return detectedColor;
}


// ColorSensorController::COLOR ColorSensorController::convertHsvToColor(HSV& hsv)
// {
// 
//   // Logger::printfLog(Logger::DEBUG, "ColorSensorController:h[%lf]s[%lf]v[%lf]", hsv.h, hsv.s, hsv.v);
//   // 明度が極端に低ければ、黒を返す
//   if(hsv.v < BLACK_LIMIT_BORDER) return COLOR::BLACK;
//   // 明度が極端に高ければ、白を返す
//   if(hsv.v > WHITE_LIMIT_BORDER) return COLOR::WHITE;
//   // 彩度が低い場合
//   if(hsv.s < SATURATION_BORDER) {
//     // 明度が低ければ、黒を返す
//     if(hsv.v < BLACK_BORDER) return COLOR::BLACK;
//     // 黒より明るく白より暗い無彩色は、灰色として区別する。
//     if(hsv.v < GRAY_VALUE_BORDER) return COLOR::GRAY;
//     // 明度が高ければ、白を返す
//     return COLOR::WHITE;
//   }
//   // この判定保留はSATURATION_BORDER以上の暗色だけに適用する。
//   if(hsv.v < CHROMATIC_VALUE_BORDER) return COLOR::NONE;
//   // 各色相の境界によって、色を判別する
//   if(hsv.h < RED_BORDER) return COLOR::RED;
//   if(hsv.h < YELLOW_BORDER) return COLOR::YELLOW;
//   if(hsv.h < GREEN_BORDER) return COLOR::GREEN;
//   if(hsv.h < BLUE_BORDER) return COLOR::BLUE;
//   return COLOR::RED;
// }

int ColorSensorController::getReflectance()
{
  return colorSensor.getReflection();
}

int ColorSensorController::getAmbient()
{
  return colorSensor.getAmbient();
}

void ColorSensorController::getRawRGB(RGB& rgb)
{
  spikeapi::ColorSensor::RGB rawRgb;
  colorSensor.getRGB(rawRgb);
  rgb.r = rawRgb.r;
  rgb.g = rawRgb.g;
  rgb.b = rawRgb.b;
}

void ColorSensorController::getRawHSV(HSV& hsv, bool surface)
{
  spikeapi::ColorSensor::HSV rawHsv;
  colorSensor.getHSV(rawHsv, surface);
  hsv.h = rawHsv.h;
  hsv.s = rawHsv.s;
  hsv.v = rawHsv.v;
}

const char* ColorSensorController::getColor(HSV& hsv)
{
  return ColorSensorController::convertColorToString(ColorSensorController::convertHsvToColor(hsv));
}
