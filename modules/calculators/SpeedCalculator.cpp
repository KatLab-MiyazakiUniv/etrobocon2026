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
  : robot(_robot),
    rightPid(_rightPid.kp, _rightPid.ki, _rightPid.kd, _targetSpeed),
    leftPid(_leftPid.kp, _leftPid.ki, _leftPid.kd, _targetSpeed),
    targetSpeed(_targetSpeed)

{
  double currentTime = ClockUtil::now();  // 現在の時間を取得(ミリ秒)
  prevRightTime = currentTime;
  prevLeftTime = currentTime;
}

double SpeedCalculator::calculateRightMotorPower()
{
  rightMotorPower
      = robot.getWheelMotorControllerInstance().getRightPower();  // rightMotorPowerの初期化
  // 走行時間を算出
  double currentRightTime = ClockUtil::now();                        // 現在の時間を取得(ミリ秒)
  double diffRightTime = (currentRightTime - prevRightTime) / 1000;  // 秒単位に変換
  // 右タイヤの走行速度を算出
  double currentRightSpeed = robot.getWheelMotorControllerInstance().getRightSpeed();
  // 走行速度に相当する右タイヤのPower値を算出
  rightMotorPower += rightPid.calculatePid(currentRightSpeed, diffRightTime);
  // メンバを更新
  prevRightTime = currentRightTime;

  return rightMotorPower;
}

double SpeedCalculator::calculateLeftMotorPower()
{
  leftMotorPower
      = robot.getWheelMotorControllerInstance().getLeftPower();  // leftMotorPowerの初期化
  // 走行時間を算出
  double currentLeftTime = ClockUtil::now();                      // 現在の時間を取得(ミリ秒)
  double diffLeftTime = (currentLeftTime - prevLeftTime) / 1000;  // 秒単位に変換
  // 左タイヤの走行速度を算出
  double currentLeftSpeed = robot.getWheelMotorControllerInstance().getLeftSpeed();
  // 走行速度に相当する左タイヤのPower値を算出
  leftMotorPower += leftPid.calculatePid(currentLeftSpeed, diffLeftTime);
  // メンバを更新
  prevLeftTime = currentLeftTime;

  return leftMotorPower;
}