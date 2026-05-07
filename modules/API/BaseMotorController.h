/**
 * @file BaseMotorController.h
 * @brief モータ制御に用いる関数をまとめたラッパークラスの親クラス
 * * @author sadomiya-sousi
 */
#ifndef Base_Motor_Controller_H
#define Base_Motor_Controller_H

#include "Motor.h"
#include "SystemInfo.h"

// using namespace spikeapi;

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

 private:
};

#endif
