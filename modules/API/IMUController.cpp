/**
 * @file IMUController.cpp
 * @brief IMUセンサを制御するラッパークラス
 * @author yutaro-1214
 */

#include "IMUController.h"

using namespace spikeapi;

namespace {

  /**
   * @brief IMU方位角の倍率補正係数
   */
  constexpr float AZIMUTH_SCALE = 1.0f;

}  // namespace

IMUController::IMUController() : imu() {}

/**
 * @brief 倍率補正後のIMU方位角を取得する
 * @return 補正後の方位角[deg]
 */
float IMUController::getAzimuth()
{
  return getRawAzimuth() * AZIMUTH_SCALE;
}

/**
 * @brief 倍率補正前のIMU方位角を取得する
 * @return 補正前の方位角[deg]
 */
float IMUController::getRawAzimuth()
{
  return imu.getHeading();
}

/**
 * @brief IMUの方位角を0度にリセットする
 */
void IMUController::resetAzimuth()
{
  imu.resetHeading();
}

/**
 * @brief IMUから加速度を取得する
 * @param accel 加速度を格納する構造体
 */
void IMUController::getAcceleration(Acceleration& accel)
{
  IMU::Acceleration acceleration;
  imu.getAcceleration(acceleration);
  accel.x = acceleration.x;
  accel.y = acceleration.y;
  accel.z = acceleration.z;
}

/**
 * @brief IMUから角速度を取得する
 * @param ang 角速度を格納する構造体
 */
void IMUController::getAngularVelocity(AngularVelocity& ang)
{
  IMU::AngularVelocity angularvelocity;
  imu.getAngularVelocity(angularvelocity);
  ang.x = angularvelocity.x;
  ang.y = angularvelocity.y;
  ang.z = angularvelocity.z;
}

/**
 * @brief IMUモジュールが使用可能かチェックする
 * @return true 準備完了、false 準備未完了
 */
bool IMUController::isReady() const
{
  return imu.isReady();
}

/**
 * @brief IMUモジュールが静止状態かチェックする
 * @return true 静止状態、false 動作中
 */
bool IMUController::isStationary() const
{
  return imu.isStationary();
}

/**
 * @brief IMUモジュールの傾斜角度をセットする
 * @param angle 傾斜角度[deg]
 */
void IMUController::setTilt(float angle)
{
  imu.setTilt(angle);
}