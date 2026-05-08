/**
 * @file   Robot.cpp
 * @brief  走行システム全体で再利用する外部リソースを管理するクラス
 * @author takuchi17
 */

#include "Robot.h"

Robot::Robot() : wheelMotorController(), armMotorController() /*, imuController()*/ {}

WheelMotorController& Robot::getWheelMotorControllerInstance()
{
  return wheelMotorController;
}

ArmMotorController& Robot::getArmMotorControllerInstance()
{
  return armMotorController;
}

// IMUController& Robot::getIMUControllerInstance()
// {
//   return imuController;
// }