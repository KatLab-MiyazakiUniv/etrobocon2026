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

void IMUController::resetAzimuth()
{
  imu.resetHeading();
}

void IMUController::getAcceleration(float (&accel)[3])
{
  IMU::Acceleration acceleration;
  imu.getAcceleration(acceleration);
  accel[0] = acceleration.x;
  accel[1] = acceleration.y;
  accel[2] = acceleration.z;
}

void IMUController::getAngularVelocity(float (&ang)[3])
{
  IMU::AngularVelocity angularvelocity;
  imu.getAngularVelocity(angularvelocity);
  ang[0] = angularvelocity.x;
  ang[1] = angularvelocity.y;
  ang[2] = angularvelocity.z;
}

bool IMUController::isReady() const
{
  return imu.isReady();
}

bool IMUController::isStationary() const
{
  return imu.isStationary();
}

void IMUController::setTilt(float angle)
{
  imu.setTilt(angle);
}