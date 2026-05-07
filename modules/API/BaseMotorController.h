/**
 * @file BaseMotorController.h
 * @brief モータ制御に用いる関数をまとめたラッパークラスの親クラス
 * @author sadomiya-sousi
 */
#ifndef BASE_MOTOR_CONTROLLER_H
#define BASE_MOTOR_CONTROLLER_H

#include "Motor.h"
#include "SystemInfo.h"  // WHEEL_RADIUS, PI, RAD_TO_DEG, DEG_TO_RADの定義を含む

class BaseMotorController {
 protected:
  /**
   * コンストラクタ
   */
  BaseMotorController();

  /**
   * @brief モータに設定するpower値の制限
   * @param inputpower 入力されたpower値
   * @return 制限されたpower値
   */
  int limitPowerValue(int inputPower);
};

#endif
