/**
 *  @file SpeedCalculator.cpp
 *  @brief 目標速度に対応するpower値を算出するクラス
 *  @author migaku2645
 */
#include "SpeedCalculator.h"
#include "ClockUtil.h"

const Pid::PidGain SpeedCalculator::DEFAULT_RIGHT_PID = { 0.016, 0.005, 0.0015 };
const Pid::PidGain SpeedCalculator::DEFAULT_LEFT_PID = { 0.016, 0.0045, 0.0015 };

SpeedCalculator::SpeedCalculator(Robot& _robot, double _targetSpeed)
  : SpeedCalculator(_robot, DEFAULT_RIGHT_PID, DEFAULT_LEFT_PID, _targetSpeed)
{
}

SpeedCalculator::SpeedCalculator(Robot& _robot, const Pid::PidGain& _rightPid,
                                 const Pid::PidGain& _leftPid, double _targetSpeed)
  : targetSpeed(_targetSpeed),
    rightPid(_rightPid.kp, _rightPid.ki, _rightPid.kd, _targetSpeed),
    leftPid(_leftPid.kp, _leftPid.ki, _leftPid.kd, _targetSpeed),
    rightMotorPower(0.0),
    leftMotorPower(0.0),
    isInitialized(false),
    robot(_robot)
{
}

void SpeedCalculator::prepare()
{
  rightPid.prepare();
  leftPid.prepare();

  // モータの現在のパワーの平均値をベース初期値とする（左右偏差をリセット）
  double avgPower = (robot.getWheelMotorControllerInstance().getRightPower()
                     + robot.getWheelMotorControllerInstance().getLeftPower())
                    / 2.0;
  rightMotorPower = avgPower;
  leftMotorPower = avgPower;
  isInitialized = true;
}

double SpeedCalculator::calculateRightMotorPower()
{
  if(!isInitialized) {
    prepare();
  }
  // 右タイヤの走行速度を算出
  double currentRightSpeed = robot.getWheelMotorControllerInstance().getRightSpeed();
  // 走行速度に相当する右タイヤのPower値を算出
  rightMotorPower += rightPid.calculatePid(currentRightSpeed);

  return rightMotorPower;
}

double SpeedCalculator::calculateLeftMotorPower()
{
  if(!isInitialized) {
    prepare();
  }
  // 左タイヤの走行速度を算出
  double currentLeftSpeed = robot.getWheelMotorControllerInstance().getLeftSpeed();
  // 走行速度に相当する左タイヤのPower値を算出
  leftMotorPower += leftPid.calculatePid(currentLeftSpeed);

  return leftMotorPower;
}
