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
#include "ColorSensorController.h"
#include "Course.h"
#include "SocketClient.h"

class Robot {
 public:
  /**
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
  IMUController& getIMUControllerInstance();
  /**
   * @brief コースの参照を返す
   * @return メンバ変数 course(Course のインスタンス) の参照
   */
  Course& getCourse();

  /**
   * @brief コースを設定する
   * @param course コース
   */
  void setCourse(Course course);

  /**
   * @brief ColorSensorControllerのインスタンスの参照を返す
   * @return メンバ変数 colorSensorController(ColorSensorController のインスタンス) の参照
   */
  ColorSensorController& getColorSensorControllerInstance();

 

 private:
  WheelMotorController wheelMotorController;    // WheelMotorController インスタンス
  ArmMotorController armMotorController;        // ArmMotorController インスタンス
  IMUController imuController;                  // IMUController インスタンス
  Course course;                                // コース(Left or Right)
  ColorSensorController colorSensorController;  // ColorSensorController インスタンス
};
#endif