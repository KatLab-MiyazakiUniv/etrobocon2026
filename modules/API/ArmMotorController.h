/**
 * @file ArmMotorController.h
 * @brief アームモータ制御に用いる関数をまとめたラッパークラス
 * @author sadomiya-sousi
 */
#ifndef ARM_MOTOR_CONTROLLER_H
#define ARM_MOTOR_CONTROLLER_H

#include "BaseMotorController.h"  // WHEEL_RADIUS, PI, RAD_TO_DEG, DEG_TO_RADの定義を含む

class ArmMotorController : public BaseMotorController {
 public:
  /**
   * コンストラクタ
   */
  ArmMotorController();

  /**
   * @brief アームモータにpower値をセット
   * @param power power値
   */
  void setArmMotorPower(int power);

  /**
   * @brief アームモータのpower値を0にリセット
   */
  void resetArmMotorPower();

  /**
   * @brief アームモータを停止する
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
};
#endif
