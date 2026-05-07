/*
 * @file WheelMotorController.cpp
 * @brief タイヤモータ制御に用いる関数をまとめたラッパークラス
 * @author sadomiya-sousi
 */

#include "WheelMotorController.h"

WheelMotorController::WheelMotorController()
  : BaseMotorController(),
    rightWheel(EPort::PORT_A),
    leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE)
{
}

// 右モータにpower値をセット
void WheelMotorController::setRightPower(int power)
{
  rightWheel.setPower(limitPowerValue(power));
}

// 左モータにpower値をセット
void WheelMotorController::setLeftPower(int power)
{
  leftWheel.setPower(limitPowerValue(power));
}

// 右モータのpower値をリセット
void WheelMotorController::resetRightPower()
{
  rightWheel.setPower(0);
}

// 左モータのpower値をリセット
void WheelMotorController::resetLeftPower()
{
  leftWheel.setPower(0);
}

// 右左両モータの状態をリセット
void WheelMotorController::resetBothPower()
{
  rightWheel.setPower(0);
  leftWheel.setPower(0);
}

// 右タイヤモータに,線速度を回転速度に変換しセットする
void WheelMotorController::setRightSpeed(double linearSpeedMmPerSec)
{
  int rightAngleSpeed = static_cast<int>(linearSpeedMmPerSec / WHEEL_RADIUS * (RAD_TO_DEG));
  rightWheel.setSpeed(rightAngleSpeed);
}

// 左タイヤモータに,線速度を回転速度に変換しセットする
void WheelMotorController::setLeftSpeed(double linearSpeedMmPerSec)
{
  int leftAngleSpeed = static_cast<int>(linearSpeedMmPerSec / WHEEL_RADIUS * (RAD_TO_DEG));
  leftWheel.setSpeed(leftAngleSpeed);
}

// 両タイヤモータを停止する
void WheelMotorController::stopBoth()
{
  rightWheel.stop();
  leftWheel.stop();
}

// ブレーキをかけてタイヤモータを停止する
void WheelMotorController::brakeBoth()
{
  rightWheel.brake();
  leftWheel.brake();
}

// 右タイヤpower値を取得する
int WheelMotorController::getRightPower()
{
  return rightWheel.getPower();
}

// 左タイヤpower値を取得する
int WheelMotorController::getLeftPower()
{
  return leftWheel.getPower();
}

// 右モータ角位置を取得する
int32_t WheelMotorController::getRightCount()
{
  return rightWheel.getCount();
}

// 左モータ角位置を取得する
int32_t WheelMotorController::getLeftCount()
{
  return leftWheel.getCount();
}

// 右タイヤモータの線速度を取得する
double WheelMotorController::getRightSpeed()
{
  double rightSpeed = rightWheel.getSpeed() * DEG_TO_RAD * WHEEL_RADIUS;
  return rightSpeed;
}

// 左タイヤモータの線速度を取得する
double WheelMotorController::getLeftSpeed()
{
  double leftSpeed = leftWheel.getSpeed() * DEG_TO_RAD * WHEEL_RADIUS;
  return leftSpeed;
}
