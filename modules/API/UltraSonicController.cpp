/**
 * @file UltraSonicController.cpp
 * @brief 超音波センサーを操作するクラス
 * @author nishijima515
 */

#include "UltraSonicController.h"

using namespace spikeapi;

UltraSonicController::UltraSonicController() : ultrasonicSensor(EPort::PORT_F) {}

int UltraSonicController::getDistance()
{
  return static_cast<int>(ultrasonicSensor.getDistance());
}