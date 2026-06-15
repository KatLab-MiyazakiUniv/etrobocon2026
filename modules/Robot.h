
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

#include "Button.h"
#include "ForceSensor.h"
#include "Display.h"

class Robot {
 public:
  /**
   * @brief 外部リソースのインスタンスを初期化する
   */
  Robot(SocketClient& _cameraSocketClient);

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
   * @brief ColorSensorControllerのインスタンスの参照を返す
   * @return メンバ変数 colorSensorController(ColorSensorController のインスタンス) の参照
   */
  ColorSensorController& getColorSensorControllerInstance();

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
   * @brief Buttonのインスタンスの参照を返す
   * @return メンバ変数button(Buttonのインスタンス)の参照
   */
  spikeapi::Button& getButtonInstance();
  /**
   * @brief ForceSensorのインスタンスの参照を返す
   * @return メンバ変数forceSensor(ForceSensorのインスタンス)の参照
   */
  spikeapi::ForceSensor& getForceSensorInstance();
  /**
   * @brief Displayのインスタンスの参照を返す
   * @return メンバ変数display(Displayのインスタンス)の参照
   */
  spikeapi::Display& getDisplayInstance();

  /**
   * @brief カメラサーバー用のSocketClientのインスタンスの参照を返す
   * @return メンバ変数
   * socketClient(デフォルト引数でDIしたカメラサーバー用のSocketClientのインスタンス) の参照
   */
  SocketClient& getCameraSocketClientInstance();

 private:
  WheelMotorController wheelMotorController;    // WheelMotorController インスタンス
  ArmMotorController armMotorController;        // ArmMotorController インスタンス
  IMUController imuController;                  // IMUController インスタンス
  Course course;                                // コース(Left or Right)
  ColorSensorController colorSensorController;  // ColorSensorController インスタンス
  SocketClient& cameraSocketClient;             // カメラサーバー用の SocketClient インスタンス



  spikeapi::Button button;            // Buttonインスタンス
  spikeapi::ForceSensor forceSensor;  // ForceSensorインスタンス
  spikeapi::Display display;          // Displayインスタンス






};
#endif