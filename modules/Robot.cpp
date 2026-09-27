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
    button(),
    forceSensor(EPort::PORT_D),
    display()
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

void Robot::setRunningStartTime()
{
  runningStartTime = ClockUtil::now();
}

spikeapi::Button& Robot::getButtonInstance()
{
  return button;
}

spikeapi::ForceSensor& Robot::getForceSensorInstance()
{
  return forceSensor;
}

spikeapi::Display& Robot::getDisplayInstance()
{
  return display;
}

void Robot::setDecryptionKey(const char* key)
{
  std::strncpy(decryptionKey, key, 4);
  decryptionKey[4] = '\0';
}

const char* Robot::getDecryptionKey() const
{
  return decryptionKey;
}

void Robot::setTargetBrightness(int brightness)
{
  targetBrightness = brightness;
}

int Robot::getTargetBrightness() const
{
  return targetBrightness;
}

void Robot::setIndexOfLabel(int _indexOfLabel)
{
  indexOfLabel = _indexOfLabel;
}

int Robot::getIndexOfLabel() const
{
  return indexOfLabel;
}

MapData& Robot::getMapDataInstance()
{
  return mapData;
}
bool Robot::getHaveQR1contents() const
{
  return haveQR1contents;
}

bool Robot::getHaveQR2contents() const
{
  return haveQR2contents;
}

void Robot::setHaveQR1contents()
{
  haveQR1contents = true;
}

void Robot::setHaveQR2contents()
{
  haveQR2contents = true;
}