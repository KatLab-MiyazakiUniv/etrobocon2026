/**
 * @file SpeedCalculator.h
 * @brief 目標速度に対応するpower値を算出するクラス
 * @author migaku2645
 */

#ifndef SPEED_CALCULATOR_H
#define SPEED_CALCULATOR_H

#include "Pid.h"
#include "Robot.h"
#include "SystemInfo.h"

class SpeedCalculator {
 public:
  /**
   * @brief speedCalculatorのコンストラクタ
   * @param _robot Robotクラスのインスタンス
   * @param _rightPid 右タイヤのPIDゲイン
   * @param _leftPid 左タイヤのPIDゲイン
   * @param _targetSpeed 目標とするタイヤ走行速度(mm/ミリ秒)
   */
  SpeedCalculator(Robot& _robot, const Pid::PidGain& _rightPid, const Pid::PidGain& _leftPid,
                  double _targetSpeed);

  /**
   * @brief 目標とする走行速度に相当する右車輪のPower値を算出する
   * @return 走行速度に相当する右タイヤのPower値
   */
  double calculateRightMotorPower();

  /**
   * @brief 目標とする走行速度に相当する左車輪のPower値を算出する
   * @return 走行速度に相当する左タイヤのPower値
   */
  double calculateLeftMotorPower();

 private:
  const double targetSpeed;  // 目標速度(mm/ミリ秒)
  Pid rightPid;              // 目標速度に対する右車輪のPID制御を行うクラスのインスタンス
  Pid leftPid;               // 目標速度に対する左車輪のPID制御を行うクラスのインスタンス
  double rightMotorPower;    // 右車輪のPower値
  double leftMotorPower;     // 左車輪のPower値
  double prevRightTime;      // 前回の右車輪の速度計算を行った時間(ミリ秒)
  double prevLeftTime;       // 前回の左車輪の速度計算を行った時間(ミリ秒)
  Robot& robot;              // Robotクラスのインスタンスの参照
};
#endif