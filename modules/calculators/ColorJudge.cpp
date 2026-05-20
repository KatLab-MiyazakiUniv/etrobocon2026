/**
 * @file ColorJudge.cpp
 * @brief 色識別クラス
 * @author sadomiya-sousi
 */

#include "ColorJudge.h"

// 呼び出すときは,文字列で色を指定して呼び出すはず>仮引数の型はstd::string>switch文ではなくif文で実装するのがふさわしいか,,>これは>staticにして,インスタンスなしで呼び出せても良くない?>.hでstaticにしてるわ
COLOR ColorJudge::convertStringToColor(std::string_view str)
{
  if(str == "BLACK") return COLOR::BLACK;
  if(str == "WHITE") return COLOR::WHITE;
  if(str == "BLUE") return COLOR::BLUE;
  if(str == "GREEN") return COLOR::GREEN;
  if(str == "YELLOW") return COLOR::YELLOW;
  if(str == "RED") return COLOR::RED;
  return COLOR::NONE;
}

const char* ColorJudge::convertColorToString(const COLOR& color)
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

COLOR ColorJudge::convertHsvToColor(const spikeapi::ColorSensor::HSV& hsv)
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
