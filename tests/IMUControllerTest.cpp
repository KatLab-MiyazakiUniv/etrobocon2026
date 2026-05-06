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

    IMUController::Acceleration accel = { 10, 10, 10 };
    imuController.getAcceleration(accel);

    EXPECT_FLOAT_EQ(10, accel.x);
    EXPECT_FLOAT_EQ(10, accel.y);
    EXPECT_FLOAT_EQ(10, accel.z);
  }

  // 角速度を取得できるかのテスト
  TEST(IMUControllerTest, getAngularVelocity)
  {
    IMUController imuController;

    IMUController::AngularVelocity ang = { 10, 10, 10 };
    imuController.getAngularVelocity(ang);

    EXPECT_FLOAT_EQ(10, ang.x);
    EXPECT_FLOAT_EQ(10, ang.y);
    EXPECT_FLOAT_EQ(10, ang.z);
  }

  // IMUモジュールが使用可能な状態かを調べられるかのテスト
  TEST(IMUControllerTest, isReady)
  {
    IMUController imuController;

    EXPECT_EQ(false, imuController.isReady());
  }

  // IMUモジュールが静止状態かどうかを調べられるかのテスト
  TEST(IMUControllerTest, isStationary)
  {
    IMUController imuController;

    EXPECT_EQ(false, imuController.isStationary());
  }

  // IMUモジュールの傾斜角度をセットできるかのテスト
  TEST(IMUControllerTest, setTilt)
  {
    IMUController imuController;

    imuController.setTilt(45);

    SUCCEED();
    // 公式APIにgetTiltがなくセットできているかチェックできないため
  }
}  // namespace IMUController_test