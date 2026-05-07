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

// /**
//  * @file MotorController.h
//  * @brief モータ制御に用いる関数をまとめたラッパークラスの親クラス
//  * * @author sadomiya-sousi
//  */
// #ifndef MOTOR_CONTROLLER_H
// #define MOTOR_CONTROLLER_H

// #include "Motor.h"
// #include "SystemInfo.h"

// // static constexpr double PI = 3.1415926;          // 円周率
// // static constexpr double RAD_TO_DEG = 180.0 / PI; // ラジアンを度数に変換するための係数
// // static constexpr double DEG_TO_RAD = PI / 180.0; // 度数をラジアンに変換するための係数
// static constexpr double WHEEL_RADIUS = 28.0; // 車輪の半径[mm]
// static constexpr int MOTOR_POWER_MAX = 100;  // Power値の上限
// static constexpr int MOTOR_POWER_MIN = -100; // Power値の下限

// #endif

// // using namespace spikeapi;

// class BaseMotorController
// {
// protected:
//     /**
//      * コンストラクタ
//      */
//     BaseMotorController();

//     /**
//      * @brief モータに設定するpower値の制限
//      * @param inputpower 入力されたpower値
//      * @return 制限されたpower値
//      */
//     int limitPowerValue(int inputPower);

//     /**
//      * @brief モータにPower値をセット
//      * @param power Power値
//      */
//     void setMotorPower(int power);

//     /**
//      * @brief モータのpower値を0にリセット
//      */
//     void resetMotorPower();

//     // /**
//     //  * @brief タイヤモータを停止する(実装時に両タイヤから片方に変更する必要あり)
//     //  */
//     void stopMotor();

//     /**
//      * @brief ブレーキをかけてモータを停止する(実装時に両タイヤから片方に変更する必要あり)
//      */
//     void brakeWheelsMotor();

//     /**
//      * @brief モータのpower値を取得する
//      * @return モータのpower値
//      */
//     int getMotorPower();

// private:
// };

// #endif

// /**
//  *@file MotorController.h
//  *@brief MotorControllerクラスの定数の定義情報を統一したファイル
//  *@author sadomiya-sousi
//  */

// #ifndef MOTOR_CONTROLLER_H
// #define MOTOR_CONTROLLER_H

// static constexpr double PI = 3.1415926;          // 円周率
// static constexpr double RAD_TO_DEG = 180.0 / PI; // ラジアンを度数に変換するための係数
// static constexpr double DEG_TO_RAD = PI / 180.0; // 度数をラジアンに変換するための係数
// static constexpr double WHEEL_RADIUS = 28.0;     // 車輪の半径[mm]
// /** Power値の上限 */
// static constexpr int MOTOR_POWER_MAX = 100;
// /** Power値の下限 */
// static constexpr int MOTOR_POWER_MIN = -100;

// #endif
