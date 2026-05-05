/**
 * @file ArmMotorController.h
 * @brief アームのモータ制御に用いる関数をまとめたラッパークラス
 * @author sadomiya-sousi
 */
#ifndef ARM_MOTOR_CONTROLLER_H
#define ARM_MOTOR_CONTROLLER_H

#include "Motor.h"
// #include "SystemInfo.h"  // WHEEL_RADIUS, PI, RAD_TO_DEG, DEG_TO_RADの定義を含む
#include "MotorSystemInfo.h"  // WHEEL_RADIUS, PI, RAD_TO_DEG, DEG_TO_RADの定義を含む

class ArmMotorController {
 public:
  /** Power値の上限 */
  static constexpr int MOTOR_POWER_MAX = 100;

  /** Power値の下限 */
  static constexpr int MOTOR_POWER_MIN = -100;

  /**
   * コンストラクタ
   */
  ArmMotorController();

  /**
   * @brief アームのモータにpower値をセット
   * @param power power値
   */
  void setArmMotorPower(int power);

  /**
   * @brief アームのモータのpower値を0にリセット
   */
  void resetArmMotorPower();

  /**
   * @brief アームのモータを停止する
   */
  void stopArmMotor();

  /**
   * @brief アームモータを止めて角度を維持する
   */
  void holdArmMotor();

  /**
   * @brief アームモータの角位置を取得する
   * @return アームモータの角位置（°）
   */
  int32_t getArmMotorCount();

  /**
   * @brief アームモータのpower値を取得する
   * @return アームモータのpower値
   */
  int getArmMotorPower();

 private:
  spikeapi::Motor armMotor;  // アームモータのインスタンス

  /**
   * @brief モータに設定するpower値の制限
   * @param inputpower 入力されたpower値
   * @return 制限されたpower値
   */
  int limitPowerValue(int inputPower);
};
#endif