/**
 * @file   Robot.cpp
 * @brief  走行システム全体で再利用する外部リソースを管理するクラス
 * @author takuchi17
 */

#include "Robot.h"

Robot::Robot(SocketClient& _cameraSocketClient)
  : wheelMotorController(),
    armMotorController(),
    imuController(),
    colorSensorController(),
    cameraSocketClient(_cameraSocketClient)
{
}

WheelMotorController& Robot::getWheelMotorControllerInstance()
{
  return wheelMotorController;
}

ArmMotorController& Robot::getArmMotorControllerInstance()
{
  return armMotorController;
}

IMUController& Robot::getIMUControllerInstance()
{
  return imuController;
}

ColorSensorController& Robot::getColorSensorControllerInstance()
{
  return colorSensorController;
}
Course& Robot::getCourse()
{
  return course;
}

SocketClient& Robot::getCameraSocketClientInstance()
{
  return cameraSocketClient;
}

void Robot::setCourse(Course course)
{
  this->course = course;
}