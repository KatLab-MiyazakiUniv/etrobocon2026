/**
 * @file IMUController.cpp
 * @brief IMUセンサを制御するラッパークラス
 * @author yutaro-1214
 */

#include "IMUController.h"

using namespace spikeapi;

IMUController::IMUController() : imu() {}

float IMUController::getAzimuth()
{
  return imu.getHeading();
}