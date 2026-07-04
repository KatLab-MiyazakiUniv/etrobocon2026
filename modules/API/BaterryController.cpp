/**
 * @file BatteryController.cpp
 * @brief バッテリーの電流と電圧を取得するクラス
 * @author migaku2645
 */
#include "BatteryController.h"

uint16_t BatteryController::getCurrent()
{
  return getBatteryInstance().getCurrent();
}

uint16_t BatteryController::getVoltage()
{
  return getBatteryInstance().getVoltage();
}

spikeapi::Battery& BatteryController::getBatteryInstance()
{
  static spikeapi::Battery battery;
  return battery;
}