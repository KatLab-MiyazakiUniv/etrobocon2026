/**
 * @file BatteryController.cpp
 * @brief バッテリーの電流と電圧を取得するクラス
 * @author migaku2645
 */
#include "BatteryController.h"

uint16_t BatteryController::getCurrent()
{
  return getBatteryIncetance().getCurrent();
}

uint16_t BatteryController::getVoltage()
{
  return getBatteryIncetance().getVoltage();
}

spikeapi::Battery& BatteryController::getBatteryIncetance()
{
  static spikeapi::Battery battery;
  return battery;
}