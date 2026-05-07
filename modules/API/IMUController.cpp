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

void IMUController::getAcceleration(Acceleration& accel)
{
  IMU::Acceleration acceleration;
  imu.getAcceleration(acceleration);
  accel.x = acceleration.x;
  accel.y = acceleration.y;
  accel.z = acceleration.z;
}

void IMUController::getAngularVelocity(AngularVelocity& ang)
{
  IMU::AngularVelocity angularvelocity;
  imu.getAngularVelocity(angularvelocity);
  ang.x = angularvelocity.x;
  ang.y = angularvelocity.y;
  ang.z = angularvelocity.z;
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