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
  void setPower(int power);

  /**
   * @brief アームモータのpower値を0にリセット
   */
  void resetPower();

  /**
   * @brief アームモータを停止する
   */
  void stopMotor();

  /**
   * @brief アームモータを止めて角度を維持する
   */
  void holdMotor();

  /**
   * @brief アームモータの角位置を取得する
   * @return アームモータの角位置（°）
   */
  int32_t getCount();

  /**
   * @brief アームモータのpower値を取得する
   * @return アームモータのpower値
   */
  int getPower();

 private:
  spikeapi::Motor armMotor;  // アームモータのインスタンス
};
#endif
