/**
 * @file SpeedCalculatorTest.cpp
 * @brief SpeedCalculatorクラスをテストする
 * @author migaku2645
 */

#include "SpeedCalculator.h"
#include <gtest/gtest.h>

namespace etrobocon2026_test {

  class SpeedCalculatorTest : public ::testing::Test {
   protected:
    // テスト用のPidゲイン
    Pid::PidGain rightPid = { 0.00535, 0.00115, 0.000 };
    Pid::PidGain leftPid = { 0.00578, 0.0008535, 0.000 };
  };

  // 左右のモーターに少しでもPower値が入っているかのテスト
  TEST_F(SpeedCalculatorTest, CalculateMotorPower)
  {
    Robot robot;
    // Powerの初期化
    robot.getWheelMotorControllerInstance().setRightPower(0.0);
    robot.getWheelMotorControllerInstance().setLeftPower(0.0);
    SpeedCalculator speedCalculator(robot, rightPid, leftPid, 300.0);
    double actualLeftPower = speedCalculator.calculateLeftMotorPower();
    double actualRightPower = speedCalculator.calculateRightMotorPower();
    double expected = 0.0;
    EXPECT_LT(expected, actualLeftPower);
    EXPECT_LT(expected, actualRightPower);
  }

  // 左右のモーターに少しでも負のPower値が入っているかのテスト
  TEST_F(SpeedCalculatorTest, CalculateMotorPowerFromMinusSpeed)
  {
    Robot robot;
    // Powerの初期化
    robot.getWheelMotorControllerInstance().setRightPower(0.0);
    robot.getWheelMotorControllerInstance().setLeftPower(0.0);
    SpeedCalculator speedCalculator(robot, rightPid, leftPid, -300.0);
    double actualLeftPower = speedCalculator.calculateLeftMotorPower();
    double actualRightPower = speedCalculator.calculateRightMotorPower();
    double expected = 0.0;
    EXPECT_GT(expected, actualLeftPower);
    EXPECT_GT(expected, actualRightPower);
  }

  // 目標速度が大きい場合に、モーターのPower値も大きくなるかのテスト
  TEST_F(SpeedCalculatorTest, CalculateMotorPowerComparison)
  {
    Robot robot;
    double smallTargetSpeed = 300.0;
    double largeTargetSpeed = 500.0;
    // Powerの初期化
    robot.getWheelMotorControllerInstance().setRightPower(0.0);
    robot.getWheelMotorControllerInstance().setLeftPower(0.0);
    SpeedCalculator speedCalculatorSmall(robot, rightPid, leftPid, smallTargetSpeed);
    double actualLeftPower = speedCalculatorSmall.calculateLeftMotorPower();
    double actualRightPower = speedCalculatorSmall.calculateRightMotorPower();

    SpeedCalculator speedCalculatorLarge(robot, rightPid, leftPid, largeTargetSpeed);
    double actualLeftPowerB = speedCalculatorLarge.calculateLeftMotorPower();
    double actualRightPowerB = speedCalculatorLarge.calculateRightMotorPower();

    EXPECT_LT(actualLeftPower, actualLeftPowerB);
    EXPECT_LT(actualRightPower, actualRightPowerB);
  }

}  // namespace etrobocon2026_test