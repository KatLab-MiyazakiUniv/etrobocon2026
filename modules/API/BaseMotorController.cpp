/**
 * @file BaseMotorController.cpp
 * @brief モータ制御に用いる関数をまとめたラッパークラスの親クラス
 * @author sadomiya-suosi
 */
#include "BaseMotorController.h"

using namespace spikeapi;

BaseMotorController::BaseMotorController() {}

// モータに設定するpower値の制限
int BaseMotorController::limitPowerValue(int inputPower)
{
  if(inputPower > MOTOR_POWER_MAX) {
    return MOTOR_POWER_MAX;
  } else if(inputPower < MOTOR_POWER_MIN) {
    return MOTOR_POWER_MIN;
  }
  return inputPower;
}

// /**
//  * @file WheelMotorController.h
//  * @brief タイヤモータ制御に用いる関数をまとめたラッパークラス
//  * @author sadomiya-sousi
//  */
// #ifndef MOTOR_CONTROLLER_H
// #define MOTOR_CONTROLLER_H

// #include "Motor.h"
// #include "MotorController.h"

// using namespace spikeapi;

// class WheelMotorController {
//  public:
//   /**
//    * コンストラクタ
//    */
//   WheelMotorController();

//   /**
//    * @brief 右タイヤモータにPower値をセット
//    * @param power Power値
//    */
//   void setRightMotorPower(int power);

//   /**
//    * @brief 左タイヤモータにPower値をセット
//    * @param power Power値
//    */
//   void setLeftMotorPower(int power);

//   /**
//    * @brief 右タイヤモータのpower値を0にリセット
//    */
//   void resetRightMotorPower();

//   /**
//    * @brief 左タイヤモータのpower値を0にリセット
//    */
//   void resetLeftMotorPower();

//   /**
//    * @brief 両タイヤモータのpower値を0にリセット
//    */
//   void resetWheelsMotorPower();

//   /**
//    * @brief 右タイヤモータに, 線速度から算出した回転速度をセット
//    * @param linearSpeedMmPerSec 線速度（mm/秒）
//    */
//   void setRightMotorSpeed(double linearSpeedMmPerSec);

//   /**
//    * @brief 左タイヤモータに, 線速度から算出した回転速度をセット
//    * @param linearSpeedMmPerSec 線速度（mm/秒）
//    */
//   void setLeftMotorSpeed(double linearSpeedMmPerSec);

//   /**
//    * @brief 両タイヤモータを停止する
//    */
//   void stopWheelsMotor();

//   /**
//    * @brief ブレーキをかけてタイヤのモータを停止する
//    */
//   void brakeWheelsMotor();

//   /**
//    * @brief 右タイヤモータの角位置を取得する
//    * @return 右タイヤモータの角位置（°）
//    */
//   int32_t getRightMotorCount();

//   /**
//    * @brief 左タイヤモータの角位置を取得する
//    * @return 左タイヤモータの角位置（°）
//    */
//   int32_t getLeftMotorCount();

//   /**
//    * @brief 右タイヤモータのpower値を取得する
//    * @return 右タイヤモータのpower値
//    */
//   int getRightMotorPower();

//   /**
//    * @brief 左タイヤモータのpower値を取得する
//    * @return 左タイヤモータのpower値
//    */
//   int getLeftMotorPower();

//   /**
//    * @brief 右タイヤモータの線速度を取得する
//    * @return 右タイヤモータの線速度（mm/秒）
//    */
//   double getRightMotorSpeed();

//   /**
//    * @brief 左タイヤモータの線速度を取得する
//    * @return 左タイヤモータの線速度（mm/秒）
//    */
//   double getLeftMotorSpeed();

//  private:
//   spikeapi::Motor rightWheel;  // 右タイヤモータのインスタンス
//   spikeapi::Motor leftWheel;   // 左タイヤモータのインスタンス

//   /**
//    * @brief モータに設定するpower値の制限
//    * @param inputpower 入力されたpower値
//    * @return 制限されたpower値
//    */
//   int limitPowerValue(int inputPower);
// };

// #endif
