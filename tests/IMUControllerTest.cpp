/**
 * @file   IMUControllerTest.cpp
 * @brief  IMUControllerクラスのテスト
 * @author yutaro-1214
 */

#include <gtest/gtest.h>
#include "IMUController.h"

namespace IMUController_test {

  // 方位角を取得できるかのテスト
  TEST(IMUControllerTest, getAzimuth)
  {
    IMUController imuController;

    EXPECT_FLOAT_EQ(90, imuController.getAzimuth());
  }

  // 方位角をリセットできるかのテスト
  TEST(IMUControllerTest, resetAzimuth)
  {
    IMUController imuController;

    imuController.resetAzimuth();

    EXPECT_FLOAT_EQ(0, imuController.getAzimuth());
  }

  // 加速度を取得できるかのテスト
  TEST(IMUControllerTest, getAcceleration)
  {
    IMUController imuController;

    float accel[3];
    accel[0] = 0;
    accel[1] = 0;
    accel[2] = 0;
    imuController.getAcceleration(accel);

    EXPECT_FLOAT_EQ(10, accel[0]);
    EXPECT_FLOAT_EQ(10, accel[1]);
    EXPECT_FLOAT_EQ(10, accel[2]);
  }

  // 角速度を取得できるかのテスト
  TEST(IMUControllerTest, getAngularVelocity)
  {
    IMUController imuController;

    float ang[3];
    ang[0] = 0;
    ang[1] = 0;
    ang[2] = 0;
    imuController.getAcceleration(ang);

    EXPECT_FLOAT_EQ(10, ang[0]);
    EXPECT_FLOAT_EQ(10, ang[1]);
    EXPECT_FLOAT_EQ(10, ang[2]);
  }

  // IMUモジュールが使用可能な状態かを調べられるかのテスト
  TEST(IMUControllerTest, isReady)
  {
    IMUController imuController;

    EXPECT_EQ(true, imuController.isReady());
  }

  // IMUモジュールが静止状態かどうかを調べられるかのテスト
  TEST(IMUControllerTest, isStationary)
  {
    IMUController imuController;

    EXPECT_EQ(true, imuController.isStationary());
  }

  // IMUモジュールの傾斜角度をセットできるかのテスト
  TEST(IMUControllerTest, setTilt)
  {
    IMUController imuController;

    spikeapi::IMU imu;
    imu.tilt = 0;

    float expected = 45;
    imuController.setTilt(45);

    EXPECT_FLOAT_EQ(expected, imu.tilt);
  }
}  // namespace IMUController_test