/**
 * @file MileageTest.cpp
 * @brief Mileageクラスをテストする
 * @author migaku2645
 */

#include <cmath>
#include <gtest/gtest.h>
#include "Mileage.h"

namespace etrobocon2026_test {
  // 右車輪と左車輪の回転角度が正の値のとき、両車輪の累計走行距離の平均が正しい値になるかのテスト
  TEST(MileageTest, CalculateMileagePlus)
  {
    int32_t rightAngle = 30;
    int32_t leftAngle = 40;

    // 計算過程
    // 1.右車輪の累計走行距離を算出
    // double rightWheelMileage = 2.0 * rightAngle * radius * M_PI / 360.0;
    // M_PI = 3.14: rightWheelMileage = 2.0 * 30.0 * 28.0 * 3.14 / 360.0 = 14.6533...
    // M_PI = 3.15: rightWheelMileage = 2.0 * 30.0 * 28.0 * 3.15 / 360.0 = 14.70
    // 2.左車輪の累計走行距離を算出
    // double leftWheelMileage = 2.0 * leftAngle * radius * M_PI / 360.0;
    // M_PI = 3.14: leftWheelMileage = 2.0 * 40.0 * 28.0 * 3.14 / 360.0 = 19.53777...
    // M_PI = 3.15: leftWheelMileage = 2.0 * 40.0 * 28.0 * 3.15 / 360.0 = 19.60
    // 3.両車輪の累計走行距離の平均を算出
    // double expected = (rightWheelMileage + leftWheelMileage) / 2.0;
    // M_PI = 3.14: (14.6533... + 19.5377...) / 2 = 17.09555...
    // M_PI = 3.15: (14.70 + 19.60) / 2 = 17.15

    double expected_min = 17.09555;
    double expected_max = 17.15;

    // actual: 17.104227
    double actual = Mileage::calculateMileage(rightAngle, leftAngle);

    // M_PIが3.14で計算した値よりも大きいアサーションと、3.15で計算した値よりも小さいアサーションを両方満たすか
    EXPECT_LT(expected_min, actual);
    EXPECT_GT(expected_max, actual);
  }

  // 右車輪と左車輪の回転角度が負の値のとき、両車輪の累計走行距離の平均が正しい値になるかのテスト
  TEST(MileageTest, CalculateMileageMinus)
  {
    int32_t rightAngle = -30;
    int32_t leftAngle = -40;

    // 計算過程
    // 1.右車輪の累計走行距離を算出
    // double rightWheelMileage = 2.0 * rightAngle * radius * M_PI / 360.0;
    // M_PI = 3.14: rightWheelMileage = 2.0 * -30.0 * 28.0 * 3.14 / 360.0 = -14.6533...
    // M_PI = 3.15: rightWheelMileage = 2.0 * -30.0 * 28.0 * 3.15 / 360.0 = -14.70
    //
    // 2.左車輪の累計走行距離を算出
    // double leftWheelMileage = 2.0 * leftAngle * radius * M_PI / 360.0;
    // M_PI = 3.14: leftWheelMileage = 2.0 * -40.0 * 28.0 * 3.14 / 360.0 = -19.5377...
    // M_PI = 3.15: leftWheelMileage = 2.0 * -40.0 * 28.0 * 3.15 / 360.0 = -19.60
    //
    // 3.両車輪の累計走行距離の平均を算出
    // double expected = (rightWheelMileage + leftWheelMileage) / 2.0;
    // M_PI = 3.14: (-14.6533... + -19.5377...) / 2 = -17.09555...
    // M_PI = 3.15: (-14.70 + -19.60) / 2 = -17.15

    double expected_min = -17.15;
    double expected_max = -17.09555;
    // actual: -17.104227
    double actual = Mileage::calculateMileage(rightAngle, leftAngle);

    // M_PIが3.15で計算した値よりも大きく、
    // 3.14で計算した値よりも小さいアサーションを両方満たすか
    EXPECT_LT(expected_min, actual);
    EXPECT_GT(expected_max, actual);
  }

  // 右車輪と左車輪の回転角度が0のとき、両車輪の累計走行距離の平均が0になるかのテスト
  TEST(MileageTest, CalculateMileageZero)
  {
    int32_t rightAngle = 0;
    int32_t leftAngle = 0;

    // 計算過程
    // 1.右車輪の累計走行距離を算出
    // double rightWheelMileage = 2.0 * rightAngle * radius * M_PI / 360.0;
    // rightWheelMileage = 0.0
    //
    // 2.左車輪の累計走行距離を算出
    // double leftWheelMileage = 2.0 * leftAngle * radius * M_PI / 360.0;
    // leftWheelMileage = 0.0
    //
    // 3.両車輪の累計走行距離の平均を算出
    // double expected = (rightWheelMileage + leftWheelMileage) / 2.0;
    // expected = (0.0 + 0.0) / 2.0 = 0.0

    double expected = 0.0;

    // actual: 0.0
    double actual = Mileage::calculateMileage(rightAngle, leftAngle);

    // 計算結果が0.0と完全一致するか
    EXPECT_DOUBLE_EQ(expected, actual);
  }
}  // namespace etrobocon2026_test