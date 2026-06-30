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
    ultraSonicController(),
    cameraSocketClient(_cameraSocketClient),
    position(),
    odometry(position),
    navigator(position)
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

UltraSonicController& Robot::getUltraSonicControllerInstance()
{
  return ultraSonicController;
}

SocketClient& Robot::getCameraSocketClientInstance()
{
  return cameraSocketClient;
}

Course& Robot::getCourse()
{
  return course;
}

void Robot::setCourse(Course course)
{
  this->course = course;
}

Edge& Robot::getEdge()
{
  return edge;
}

void Robot::setEdge(Edge edge)
{
  this->edge = edge;
}

int Robot::getRunningStartTime()
{
  return runningStartTime;
}

Position& Robot::getPosition()
{
  return position;
}

Odometry& Robot::getOdometry()
{
  return odometry;
}

Navigator& Robot::getNavigator()
{
  return navigator;
}