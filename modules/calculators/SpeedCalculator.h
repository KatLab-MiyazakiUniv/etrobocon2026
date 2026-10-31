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
#include "CsvLogger.h"

class SpeedCalculator {
 public:
  /**
   * @brief デフォルトPIDを使うコンストラクタ
   * @param _robot Robotクラスのインスタンス
   * @param _targetSpeed 目標とするタイヤ走行速度(mm/秒)
   */
  SpeedCalculator(Robot& _robot, double _targetSpeed);

  /**
   * @brief 左右タイヤのPIDを指定するオーバーロードコンストラクタ
   * @param _robot Robotクラスのインスタンス
   * @param _rightPid 右タイヤのPIDゲイン
   * @param _leftPid 左タイヤのPIDゲイン
   * @param _targetSpeed 目標とするタイヤ走行速度(mm/秒)
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

  /**
   * @brief 目標走行速度に相当するのPower値の初期値を算出
   * @return 走行速度に相当するタイヤのPower値
   */
  int getPowerFromSpeed(int speed);

 private:
  const double targetSpeed;  // 目標速度(mm/秒)
  Pid rightPid;              // 目標速度に対する右車輪のPID制御を行うクラスのインスタンス
  Pid leftPid;               // 目標速度に対する左車輪のPID制御を行うクラスのインスタンス
  double rightMotorPower;    // 右車輪のPower値
  double leftMotorPower;     // 左車輪のPower値
  Robot& robot;              // Robotクラスのインスタンスの参照
  static const Pid::PidGain DEFAULT_RIGHT_PID;  // 右車輪用のデフォルトPIDゲイン
  static const Pid::PidGain DEFAULT_LEFT_PID;   // 左車輪用のデフォルトPIDゲイン

  // 正方向の目標速度に応じたPower値定義
  static constexpr int MOTOR_POWER_AT_550 = 100;  // 550以上
  static constexpr int MOTOR_POWER_AT_500 = 50;
  static constexpr int MOTOR_POWER_AT_400 = 50;
  static constexpr int MOTOR_POWER_AT_300 = 50;
  static constexpr int MOTOR_POWER_AT_200 = 50;
  static constexpr int MOTOR_POWER_AT_100 = 50;

  // 停止
  static constexpr int MOTOR_POWER_STOP = 0;

  // 負方向（後退）の目標速度に応じたPower値定義
  static constexpr int MOTOR_POWER_AT_MINUS_100 = -50;
  static constexpr int MOTOR_POWER_AT_MINUS_200 = -50;
  static constexpr int MOTOR_POWER_AT_MINUS_300 = -50;
  static constexpr int MOTOR_POWER_AT_MINUS_400 = -50;
  static constexpr int MOTOR_POWER_AT_MINUS_500 = -50;
  static constexpr int MOTOR_POWER_AT_MINUS_550 = -100;  // -550以下
};
#endif
