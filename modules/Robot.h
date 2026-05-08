/**
 * @file   Robot.h
 * @brief  走行システム全体で再利用する外部リソースを管理するクラス
 * @author takuchi17
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "WheelMotorController.h"
#include "ArmMotorController.h"
#include "IMUController.h"

class Robot {
 public:
  /**
   * コンストラクタ
   * @brief 外部リソースのインスタンスを初期化する
   */
  Robot();

  /**
   * @brief WheelMotorControllerのインスタンスの参照を返す
   * @return メンバ変数 wheelMotorController(WheelMotorController のインスタンス) の参照
   */
  WheelMotorController& getWheelMotorControllerInstance();

  /**
   * @brief ArmMotorControllerのインスタンスの参照を返す
   * @return メンバ変数 armMotorController(ArmMotorController のインスタンス) の参照
   */
  ArmMotorController& getArmMotorControllerInstance();

  /**
   * @brief IMUControllerのインスタンスの参照を返す
   * @return メンバ変数 imuController(IMUController のインスタンス) の参照
   */
  // IMUController& getIMUControllerInstance();

 private:
  WheelMotorController wheelMotorController;  // WheelMotorController インスタンス
  ArmMotorController armMotorController;      // ArmMotorController インスタンス
  // IMUController imuController;                // IMUController インスタンス
};

#endif