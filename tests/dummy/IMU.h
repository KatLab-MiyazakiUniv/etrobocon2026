/**
 * @file IMU.h
 * @brief IMUクラス(ダミー)
 * @author yutaro-1214
 */

#ifndef SPIKE_CPP_API_IMU_H_
#define SPIKE_CPP_API_IMU_H_

#include <cstdlib>
#include <cmath>

namespace spikeapi {
  /**
   * SPIKE IMUクラス
   */
  class IMU {
   public:
    /* 加速度 mm/s^2*/
    struct Acceleration {
      float x;
      float y;
      float z;
    };
    /* 角速度 degree/s */
    struct AngularVelocity {
      float x;
      float y;
      float z;
    };

    spikeapi::IMU::Acceleration accel = { 10, 10, 10 };
    spikeapi::IMU::AngularVelocity ang = { 10, 10, 10 };
    float azimuth = 90;
    float tilt = 0;
    float angle = 0;
    int stat;
    int ready;

    /**
     * IMUから加速度を取得する
     * @param accel [out] x/y/z軸の加速度を格納するためのAcceleration構造体[mm/s^2]
     */
    void getAcceleration(spikeapi::IMU::Acceleration& accel)
    {
      accel.x = 10;
      accel.y = 10;
      accel.z = 10;
    };

    /**
     * IMUから角速度を取得する
     * @param ang [out] x/y/z軸の角速度を格納するためのAngularVelocity構造体[°/s]
     */
    void getAngularVelocity(spikeapi::IMU::AngularVelocity& ang)
    {
      ang.x = 10;
      ang.y = 10;
      ang.z = 10;
    };

    /**
     * IMUモジュールが使用可能かチェックする
     * @return true: 準備が整っている, false: まだ準備が整っていない
     */
    bool isReady() const
    {
      if(ready == 0)
        return true;
      else
        return false;
    }

    /**
     * IMUモジュールが静止状態かチェックする
     * @return true: 静止している, false: 動いている
     */
    bool isStationary() const
    {
      if(stat == 0)
        return true;
      else
        return false;
    }

    /**
     * IMUモジュールの傾斜角度をセットする
     * @param angle [in] モジュールの（Y軸に対する）傾斜角度（度）
     */
    void setTilt(float angle) { tilt = angle; }

    /**
     * IMUの方位角を取得する
     * @return 方位角 [°]
     */
    float getHeading() { return azimuth; }

    /**
     * IMUの方位角をリセットする
     */
    void resetHeading() { azimuth = 0; }
  };
}  // namespace spikeapi

#endif  // !SPIKE_CPP_API_IMU_H_