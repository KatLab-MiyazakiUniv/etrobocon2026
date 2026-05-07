/**
 * @file ArmMotorController.cpp
 * @brief アームモータ制御に用いる関数をまとめたラッパークラス
 * @author sadomiya-sousi
 */

#include "ArmMotorController.h"

using namespace spikeapi;

ArmMotorController::ArmMotorController()
  : BaseMotorController(), armMotor(EPort::PORT_C, Motor::EDirection::COUNTERCLOCKWISE)
{
}

// アームモータにpower値をセット
void ArmMotorController::setArmMotorPower(int power)
{
  armMotor.setPower(limitPowerValue(power));
}

// アームモータのpower値をリセット
void ArmMotorController::resetArmMotorPower()
{
  armMotor.setPower(0);
}

// アームモータを停止する
void ArmMotorController::stopArmMotor()
{
  armMotor.stop();
}

// アームモータを止めて角度を維持する
void ArmMotorController::holdArmMotor()
{
  armMotor.hold();
}

// アームモータのpower値を取得する
int ArmMotorController::getArmMotorPower()
{
  return armMotor.getPower();
}

// アームモータの角位置を取得する
int32_t ArmMotorController::getArmMotorCount()
{
  return armMotor.getCount();
}

// /**
//  * @file ArmMotorController.cpp
//  * @brief アームモータ制御に用いる関数をまとめたラッパークラス
//  * @author sadomiya-sousi
//  */

// #include "ArmMotorController.h"

// using namespace spikeapi;

// ArmMotorController::ArmMotorController()
//     : BaseMotorController(),
//       armMotor(EPort::PORT_C, Motor::EDirection::COUNTERCLOCKWISE)
// {
// }

// // アームモータにpower値をセット
// void ArmMotorController::setArmMotorPower(int power)
// {
//   armMotor.setPower(limitPowerValue(power));
// }

// // アームモータのpower値をリセット
// void ArmMotorController::resetArmMotorPower()
// {
//   armMotor.setPower(0);
// }

// // アームモータを停止する
// void ArmMotorController::stopArmMotor()
// {
//   armMotor.stop();
// }

// // アームモータを止めて角度を維持する
// void ArmMotorController::holdArmMotor()
// {
//   armMotor.hold();
// }

// // アームモータのpower値を取得する
// int ArmMotorController::getArmMotorPower()
// {
//   return armMotor.getPower();
// }

// // アームモータの角位置を取得する
// int32_t ArmMotorController::getArmMotorCount()
// {
//   return armMotor.getCount();
// }
