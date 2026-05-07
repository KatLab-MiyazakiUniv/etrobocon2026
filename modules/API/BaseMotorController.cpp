/**
 * @file BaseMotorController.cpp
 * @brief モータ制御に用いる関数をまとめたラッパークラスの親クラス
 * @author sadomiya-suosi
 */
#include "BaseMotorController.h"

BaseMotorController::BaseMotorController() {}

// モータに設定するpower値の制限
int BaseMotorController::limitPowerValue(int inputPower)
{
  if(inputPower > MOTOR_POWER_MAX) {
    return MOTOR_POWER_MAX;
  } else if(inputPower < MOTOR_POWER_MIN) {
    return MOTOR_POWER_MIN;
  }
  return inputPower;
}
