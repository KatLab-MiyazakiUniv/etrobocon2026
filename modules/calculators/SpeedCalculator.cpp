/**
 *  @file SpeedCalculator.cpp
 *  @brief 目標速度に対応するpower値を算出するクラス
 *  @author migaku2645
 */
#include "SpeedCalculator.h"
#include "ClockUtil.h"

const Pid::PidGain SpeedCalculator::DEFAULT_RIGHT_PID = { 0.014849, 0.004863, 0.0015 };
const Pid::PidGain SpeedCalculator::DEFAULT_LEFT_PID = { 0.01574, 0.0045, 0.0015 };

SpeedCalculator::SpeedCalculator(Robot& _robot, double _targetSpeed)
  : SpeedCalculator(_robot, DEFAULT_RIGHT_PID, DEFAULT_LEFT_PID, _targetSpeed)
{
}

SpeedCalculator::SpeedCalculator(Robot& _robot, const Pid::PidGain& _rightPid,
                                 const Pid::PidGain& _leftPid, double _targetSpeed)
  : targetSpeed(_targetSpeed),
    rightPid(_rightPid.kp, _rightPid.ki, _rightPid.kd, _targetSpeed),
    leftPid(_leftPid.kp, _leftPid.ki, _leftPid.kd, _targetSpeed),
    robot(_robot)

{
}

double SpeedCalculator::calculateRightMotorPower()
{
  rightMotorPower = getPowerFromSpeed(targetSpeed);  // rightMotorPowerの初期化
  CsvLogger::current_logData.currentRightMotorPower = rightMotorPower;

  // 右タイヤの走行速度を算出
  double currentRightSpeed = robot.getWheelMotorControllerInstance().getRightSpeed();
  CsvLogger::current_logData.currentRightSpeed = currentRightSpeed;

  // 走行速度に相当する右タイヤのPower値を算出
  rightMotorPower += rightPid.calculatePid(currentRightSpeed);
  CsvLogger::current_logData.baseRightPower = rightMotorPower;
  return rightMotorPower;
}

double SpeedCalculator::calculateLeftMotorPower()
{
  rightMotorPower = getPowerFromSpeed(targetSpeed);  // rightMotorPowerの初期化
  CsvLogger::current_logData.currentLeftMotorPower = leftMotorPower;

  // 左タイヤの走行速度を算出
  double currentLeftSpeed = robot.getWheelMotorControllerInstance().getLeftSpeed();
  CsvLogger::current_logData.currentLeftSpeed = currentLeftSpeed;

  // 走行速度に相当する左タイヤのPower値を算出
  leftMotorPower += leftPid.calculatePid(currentLeftSpeed);
  CsvLogger::current_logData.baseLeftPower = leftMotorPower;
  return leftMotorPower;
}

int SpeedCalculator::getPowerFromSpeed(int speed)
{
  if(speed >= 550) {
    return MOTOR_POWER_AT_550;
  } else if(speed >= 500) {
    return MOTOR_POWER_AT_500;
  } else if(speed >= 400) {
    return MOTOR_POWER_AT_400;
  } else if(speed >= 300) {
    return MOTOR_POWER_AT_300;
  } else if(speed >= 200) {
    return MOTOR_POWER_AT_200;
  } else if(speed >= 100) {
    return MOTOR_POWER_AT_100;
  } else if(speed <= -550) {
    return MOTOR_POWER_AT_MINUS_550;
  } else if(speed <= -500) {
    return MOTOR_POWER_AT_MINUS_500;
  } else if(speed <= -400) {
    return MOTOR_POWER_AT_MINUS_400;
  } else if(speed <= -300) {
    return MOTOR_POWER_AT_MINUS_300;
  } else if(speed <= -200) {
    return MOTOR_POWER_AT_MINUS_200;
  } else if(speed <= -100) {
    return MOTOR_POWER_AT_MINUS_100;
  }
  return MOTOR_POWER_STOP;
}
