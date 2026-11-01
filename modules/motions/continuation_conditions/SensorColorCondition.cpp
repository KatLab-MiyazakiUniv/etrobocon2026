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

  ColorSensorController::HSV hsv;
  robot.getColorSensorControllerInstance().getRawHSV(hsv, true);

  ColorSensorController::COLOR currentColor = ColorSensorController::COLOR::NONE;

  // 校正済みデータがあれば ColorClassifier を使う
  const auto* colorData = robot.getColorDataManagerInstance().getColorData(targetColor);
  if(colorData != nullptr && colorData->acquired) {
    currentColor = robot.getColorClassifierInstance().classify(hsv);
  } else {
    // 互換性のため従来の判定にフォールバック
    currentColor = robot.getColorSensorControllerInstance().convertHsvToColor(hsv);
  }

  if(currentColor == targetColor) {
    colorCount++;
  } else {
    colorCount = 0;
  }

  if(colorCount >= JUDGE_COUNT) {
    Logger::info("Target color detected");
    return false;
  }

  return true;
}