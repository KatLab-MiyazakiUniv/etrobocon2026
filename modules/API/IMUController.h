/**
 * @file IMUController.h
 * @brief IMUセンサを制御するラッパークラス
 *        (参照元 sdk\common\library\libcpp-spike\include\IMU.h)
 * @author yutaro-1214
 */

#ifndef IMU_CONTROLLER_H
#define IMU_CONTROLLER_H

#include "IMU.h"

class IMUController {
 public:
  /**
   * @brief 加速度[mm/s^2]
   */
  struct Acceleration {
    float x;
    float y;
    float z;
  };

  /**
   * @brief 角速度[deg/s]
   */
  struct AngularVelocity {
    float x;
    float y;
    float z;
  };

  /**
   * @brief コンストラクタ
   */
  IMUController();

  /**
   * @brief 倍率補正後のIMU方位角を取得する
   * @return 補正後の方位角[deg]
   */
  float getAzimuth();

  /**
   * @brief 倍率補正前のIMU方位角を取得する
   * @return 補正前の方位角[deg]
   */
  float getRawAzimuth();

  /**
   * @brief IMUの方位角を0度にリセットする
   */
  void resetAzimuth();

  /**
   * @brief IMUから加速度を取得する
   * @param accel 加速度を格納する構造体
   */
  void getAcceleration(Acceleration& accel);

  /**
   * @brief IMUから角速度を取得する
   * @param ang 角速度を格納する構造体
   */
  void getAngularVelocity(AngularVelocity& ang);

  /**
   * @brief IMUモジュールが使用可能かチェックする
   * @return true 準備完了、false 準備未完了
   */
  bool isReady() const;

  /**
   * @brief IMUモジュールが静止状態かチェックする
   * @return true 静止状態、false 動作中
   */
  bool isStationary() const;

  /**
   * @brief IMUモジュールの傾斜角度をセットする
   * @param angle 傾斜角度[deg]
   */
  void setTilt(float angle);

 private:
  spikeapi::IMU imu;  // IMUインスタンス
};

#endif  // IMU_CONTROLLER_H