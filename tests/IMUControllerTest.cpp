/**
 * @file   IMUControllerTest.cpp
 * @brief  IMUControllerクラスのテスト
 * @author yutaro-1214
 */

#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include "IMUController.h"

namespace IMUController_test {

  // 方位角を取得できるかのテスト
  TEST(IMUControllerTest, getAzimuth)
  {
    IMUController imuController;

    // ダミーではgetAzimuthで一度回転するので１度でテストする
    EXPECT_FLOAT_EQ(1, imuController.getAzimuth());
  }

  // 方位角をリセットできるかのテスト
  TEST(IMUControllerTest, resetAzimuth)
  {
    IMUController imuController;

    imuController.resetAzimuth();

    // ダミーではgetAzimuthで一度回転するので１度でテストする
    EXPECT_FLOAT_EQ(1, imuController.getAzimuth());
  }

  // 加速度を取得できるかのテスト
  TEST(IMUControllerTest, getAcceleration)
  {
    IMUController imuController;

    srand(12345);
    IMUController::Acceleration accel = { 0, 0, 0 };
    imuController.getAcceleration(accel);

    srand(12345);
    float expected_x = (float)(rand() % 180 + 10);
    float expected_y = (float)(rand() % 180);
    float expected_z = (float)(rand() % 180 - 10);

    EXPECT_FLOAT_EQ(expected_x, accel.x);
    EXPECT_FLOAT_EQ(expected_y, accel.y);
    EXPECT_FLOAT_EQ(expected_z, accel.z);
  }

  // 角速度を取得できるかのテスト
  TEST(IMUControllerTest, getAngularVelocity)
  {
    IMUController imuController;

    srand(12345);
    IMUController::AngularVelocity ang = { 0, 0, 0 };
    imuController.getAngularVelocity(ang);

    srand(12345);
    float expected_x = (float)(rand() % 180 + 20);
    float expected_y = (float)(rand() % 180);
    float expected_z = (float)(rand() % 180 - 20);

    EXPECT_FLOAT_EQ(expected_x, ang.x);
    EXPECT_FLOAT_EQ(expected_y, ang.y);
    EXPECT_FLOAT_EQ(expected_z, ang.z);
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