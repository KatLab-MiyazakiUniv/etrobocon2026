/**
 * @file   ColorContinuationCondition.cpp
 * @brief  目標色を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#include "ColorContinuationCondition.h"

ColorContinuationCondition::ColorContinuationCondition(Robot& _robot,
                                                       ColorSensorController::COLOR _targetColor)
  : BaseContinuationCondition(_robot), targetColor(_targetColor), colorCount(0)
{
  LOG_CREATE("ColorContinuationCondition");
}

ColorContinuationCondition::~ColorContinuationCondition()
{
  LOG_DESTROY("ColorContinuationCondition");
}

void ColorContinuationCondition::prepare()
{
  // 初期化処理
}

bool ColorContinuationCondition::shouldContinue()
{
  if(targetColor == ColorSensorController::COLOR::NONE) {
    return false;
  }
  // HSV値を取得
  ColorSensorController::HSV hsv;
  robot.getColorSensorControllerInstance().getRawHSV(hsv, true);
  ColorSensorController::COLOR CurrentColor
      = robot.getColorSensorControllerInstance().convertHsvToColor(hsv);
  if(CurrentColor == targetColor) {
    colorCount++;
  } else {
    colorCount = 0;
  }
  // 指定された色をJUDGE_COUNT回連続で取得したときモータが止まる
  if(colorCount >= JUDGE_COUNT) return false;

  return true;
}