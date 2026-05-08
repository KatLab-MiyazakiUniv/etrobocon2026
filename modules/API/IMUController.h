// /**
//  *@file IMUController.h
//  *@brief IMUセンサを制御するラッパークラス　(参照元 sdk\common\library\libcpp-spike\include\IMU.h)
//  *@author yutaro-1214
//  */

// #ifndef IMU_CONTROLLER_H
// #define IMU_CONTROLLER_H

// #include "IMU.h"

// class IMUController {
//  public:
//   /* 加速度 mm/s^2*/
//   struct Acceleration {
//     float x;
//     float y;
//     float z;
//   };
//   /* 角速度 degree/s */
//   struct AngularVelocity {
//     float x;
//     float y;
//     float z;
//   };

//   /**
//    * コンストラクタ
//    */
//   IMUController();

//   /**
//    * @brief IMUの方位角を取得する
//    * @return 方位角（°）
//    */
//   float getAzimuth();

//   /**
//    * @brief IMUの方位角を0°にリセットする
//    */
//   void resetAzimuth();

//   /**
//    * @brief IMUから加速度を取得する
//    * @param accel[3] x/y/z軸の加速度を格納するための構造体
//    */
//   void getAcceleration(Acceleration& accel);

//   /**
//    * @brief IMUから角速度を取得する
//    * @param ang[3] x/y/z軸の角速度を格納するための構造体
//    */
//   void getAngularVelocity(AngularVelocity& ang);

//   /**
//    * @brief IMUモジュールが使用可能かチェックする
//    * @return true:準備が整っている, false:準備が整っていない
//    */
//   bool isReady() const;

//   /**
//    * @brief IMUモジュールが静止状態かチェックする
//    * @return true: 静止している, false: 動いている
//    */
//   bool isStationary() const;

//   /**
//    * @brief IMUモジュールの傾斜角度をセットする
//    * @param angle [in] モジュールの（Y軸に対する）傾斜角度（度）
//    */
//   void setTilt(float angle);

//  private:
//   spikeapi::IMU imu;  // IMUインスタンス
// };

// #endif