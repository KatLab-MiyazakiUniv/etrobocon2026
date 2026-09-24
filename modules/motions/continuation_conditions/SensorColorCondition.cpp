/**
 * @file   SensorColorCondition.cpp
 * @brief  目標色を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#include "SensorColorCondition.h"

SensorColorCondition::SensorColorCondition(Robot& _robot, ColorSensorController::COLOR _targetColor)
  : BaseContinuationCondition(_robot), targetColor(_targetColor), colorCount(0)
{
  LOG_CREATE("SensorColorCondition");
}

SensorColorCondition::~SensorColorCondition()
{
  LOG_DESTROY("SensorColorCondition");
}

void SensorColorCondition::prepare()
{
  colorCount = 0;
}

bool SensorColorCondition::shouldContinue()
{
  if(targetColor == ColorSensorController::COLOR::NONE) {
    Logger::error("目標色が無効です。動作を終了します。");
    return false;
  }
  // HSV値を取得
  ColorSensorController::HSV hsv;
  robot.getColorSensorControllerInstance().getRawHSV(hsv, true);
  Logger::printfLog(Logger::DEBUG, "h:%ds:%dv:%d", hsv.h, hsv.s, hsv.v);

  ColorSensorController::COLOR CurrentColor
      = robot.getColorSensorControllerInstance().convertHsvToColor(hsv);

  Logger::printfLog(Logger::DEBUG, "カラーセンサーの判定は%sです",
                    ColorSensorController::convertColorToString(CurrentColor));

  if(CurrentColor == targetColor) {
    colorCount++;
  } else {
    colorCount = 0;
  }
  // 指定された色をJUDGE_COUNT回連続で取得したときモータが止まる
  if(colorCount >= JUDGE_COUNT) {
    Logger::info("Target color detected");
    return false;
  }

  return true;
}
