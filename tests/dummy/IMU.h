/**
 * @file IMU.h
 * @brief IMUクラス(ダミー)
 * @author yutaro-1214
 */

#ifndef SPIKE_CPP_API_IMU_H_
#define SPIKE_CPP_API_IMU_H_

#include <cstdlib>
#include <cmath>
#include <iostream>

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

    spikeapi::IMU::Acceleration accel = { 0, 0, 0 };
    spikeapi::IMU::AngularVelocity ang = { 0, 0, 0 };
    float azimuth = 0;
    float tilt = 0;
    float angle = 0;
    bool is_stat = false;
    bool is_ready = false;

    /**
     * コンストラクタ
     * @param -
     * @return -
     */
    IMU(void)
    {
      srand(12345);  // テスト用固定シード値
    }

    /**
     * IMUから加速度を取得する
     * @param accel [out] x/y/z軸の加速度を格納するためのAcceleration構造体[mm/s^2]
     */
    void getAcceleration(spikeapi::IMU::Acceleration& accel)
    {
      accel.x = (float)(rand() % 180 + 10);
      accel.y = (float)(rand() % 180);
      accel.z = (float)(rand() % 180 - 10);
    };

    /**
     * IMUから角速度を取得する
     * @param ang [out] x/y/z軸の角速度を格納するためのAngularVelocity構造体[°/s]
     */
    void getAngularVelocity(spikeapi::IMU::AngularVelocity& ang)
    {
      ang.x = (float)(rand() % 180 + 20);
      ang.y = (float)(rand() % 180);
      ang.z = (float)(rand() % 180 - 20);
    };

    /**
     * IMUモジュールが使用可能かチェックする
     * @return true: 準備が整っている, false: まだ準備が整っていない
     */
    bool isReady() const { return is_ready; }

    /**
     * IMUモジュールが静止状態かチェックする
     * @return true: 静止している, false: 動いている
     */
    bool isStationary() const { return is_stat; }

    /**
     * IMUモジュールの傾斜角度をセットする
     * @param angle [in] モジュールの（Y軸に対する）傾斜角度（度）
     */
    void setTilt(float angle) { tilt = angle; }

    /**
     * IMUの方位角を取得する
     * (このダミーでは実行するごとに1度ずつ回転もする)
     * @return 方位角 [°]
     */
    float getHeading()
    {
      float prevAzimuth = azimuth;  // 更新前を保持

      azimuth += 1.0f;  // 方位角を1度ずつ増加させる

      // // 角度が360度を超えた場合、0〜360度の範囲に正規化する
      if(azimuth >= 360.0f) {
        azimuth = fmodf(azimuth, 360.0f);
      }

      return prevAzimuth;  //
    }

    /**
     * IMUの方位角をリセットする
     */
    void resetHeading() { azimuth = 0; }
  };
}  // namespace spikeapi

#endif  // !SPIKE_CPP_API_IMU_H_