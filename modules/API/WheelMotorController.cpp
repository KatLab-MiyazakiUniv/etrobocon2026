/*
 * @file WheelMotorController.cpp
 * @brief タイヤモータ制御に用いる関数をまとめたラッパークラス
 * @author sadomiya-sousi
 */

#include "WheelMotorController.h"

WheelMotorController::WheelMotorController()
    : MotorController(),
      rightWheel(EPort::PORT_A),
      leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE)
{
}

// 右モータにpower値をセット
void WheelMotorController::setRightMotorPower(int power)
{
  rightWheel.setPower(limitPowerValue(power));
}

// 左モータにpower値をセット
void WheelMotorController::setLeftMotorPower(int power)
{
  leftWheel.setPower(limitPowerValue(power));
}

// 右モータのpower値をリセット
void WheelMotorController::resetRightMotorPower()
{
  rightWheel.setPower(0);
}

// 左モータのpower値をリセット
void WheelMotorController::resetLeftMotorPower()
{
  leftWheel.setPower(0);
}

// 右左両モータの状態をリセット
void WheelMotorController::resetWheelsMotorPower()
{
  rightWheel.setPower(0);
  leftWheel.setPower(0);
}

// 右タイヤモータに,線速度を回転速度に変換しセットする
void WheelMotorController::setRightMotorSpeed(double linearSpeedMmPerSec)
{
  int rightAngleSpeed = static_cast<int>(linearSpeedMmPerSec / WHEEL_RADIUS * (RAD_TO_DEG));
  rightWheel.setSpeed(rightAngleSpeed);
}

// 左タイヤモータに,線速度を回転速度に変換しセットする
void WheelMotorController::setLeftMotorSpeed(double linearSpeedMmPerSec)
{
  int leftAngleSpeed = static_cast<int>(linearSpeedMmPerSec / WHEEL_RADIUS * (RAD_TO_DEG));
  leftWheel.setSpeed(leftAngleSpeed);
}

// 両タイヤモータを停止する
void WheelMotorController::stopWheelsMotor()
{
  rightWheel.stop();
  leftWheel.stop();
}

// ブレーキをかけてタイヤモータを停止する
void WheelMotorController::brakeWheelsMotor()
{
  rightWheel.brake();
  leftWheel.brake();
}

// 右タイヤpower値を取得する
int WheelMotorController::getRightMotorPower()
{
  return rightWheel.getPower();
}

// 左タイヤpower値を取得する
int WheelMotorController::getLeftMotorPower()
{
  return leftWheel.getPower();
}

// 右モータ角位置を取得する
int32_t WheelMotorController::getRightMotorCount()
{
  return rightWheel.getCount();
}

// 左モータ角位置を取得する
int32_t WheelMotorController::getLeftMotorCount()
{
  return leftWheel.getCount();
}

// 右タイヤモータの線速度を取得する
double WheelMotorController::getRightMotorSpeed()
{
  double rightSpeed = rightWheel.getSpeed() * DEG_TO_RAD * WHEEL_RADIUS;
  return rightSpeed;
}

// 左タイヤモータの線速度を取得する
double WheelMotorController::getLeftMotorSpeed()
{
  double leftSpeed = leftWheel.getSpeed() * DEG_TO_RAD * WHEEL_RADIUS;
  return leftSpeed;
}