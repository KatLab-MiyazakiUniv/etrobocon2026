#include "BatteryController.h"

uint16_t BatteryController::getCurrent()
{
  return getBatteryIncetance().getCurrent();
}

uint16_t BatteryController::getVoltage()
{
  return getBatteryIncetance().getVoltage();
}

Battery& BatteryController::getBatteryIncetance()
{
  static spikeapi::Battery battery;
  return battery;
}
