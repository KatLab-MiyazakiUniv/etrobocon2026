#include "ColorSensorDebug.h"
#include "ColorSensorController.h"

ColorSensorDebug::ColorSensorDebug(
    Robot& _robot,
    std::unique_ptr<BaseContinuationCondition> _continuationCondition)
  : BaseMotion(_robot, std::move(_continuationCondition))
{
  LOG_CREATE("ColorSensorDebug");
}

ColorSensorDebug::~ColorSensorDebug()
{
  LOG_DESTROY("ColorSensorDebug");
}

bool ColorSensorDebug::canStart()
{
  return true;
}

void ColorSensorDebug::executeStep()
{
  auto& colorSensor =
      robot.getColorSensorControllerInstance();

  ColorSensorController::HSV hsv;

  // カラーセンサーから生のHSV値を取得する。
  colorSensor.getRawHSV(hsv);

  // HSV値と判定された色を出力する。
  auto color = ColorSensorController::convertHsvToColor(hsv);

  Logger::printfLog(
      Logger::INFO,
      "[ColorSensorDebug] H=%u, S=%u, V=%u, Color=%s",
      static_cast<unsigned int>(hsv.h),
      static_cast<unsigned int>(hsv.s),
      static_cast<unsigned int>(hsv.v),
      ColorSensorController::convertColorToString(color));
}

void ColorSensorDebug::finish()
{
  Logger::info("ColorSensorDebug: 終了");
}