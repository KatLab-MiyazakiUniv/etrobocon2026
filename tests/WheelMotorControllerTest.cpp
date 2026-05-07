/**
 * @file WheelMotorControllerTest.cpp
 * @brief WheelMotorControllerクラスをテストする
 * @author sadomiya-sousi
 */

#include "gtest/gtest.h"
#include "WheelMotorController.h"

using namespace spikeapi;

namespace etrobocon2026_test {

  // 右モータにPower値をセットできるかのテスト
  TEST(WheelMotorControllerTest, SetRightPower)
  {
    WheelMotorController wheelController;
    wheelController.setRightPower(50);
    EXPECT_EQ(50, wheelController.getRightPower());
  }

  // 右モータにマイナスのPower値をセットできるかどうかのテスト
  TEST(WheelMotorControllerTest, SetminusPower)
  {
    WheelMotorController wheelController;
    wheelController.setRightPower(-50);
    EXPECT_EQ(-50, wheelController.getRightPower());
  }

  // 左モータにPower値をセットできるかのテスト
  TEST(WheelMotorControllerTest, SetLeftPower)
  {
    WheelMotorController wheelController;
    wheelController.setLeftPower(50);
    EXPECT_EQ(50, wheelController.getLeftPower());
  }

  // 左モータにマイナスのPower値をセットできるかどうかのテスト
  TEST(WheelMotorControllerTest, SetLeftMotorMinusPower)
  {
    WheelMotorController wheelController;
    wheelController.setLeftPower(-50);
    EXPECT_EQ(-50, wheelController.getLeftPower());
  }

  // モータに設定するPower値の下限の制限が行われているか確認するテスト
  TEST(WheelMotorControllerTest, SetLeftMotorPowerMin)
  {
    WheelMotorController wheelController;

    wheelController.setLeftPower(-150);
    EXPECT_EQ(MOTOR_POWER_MIN, wheelController.getLeftPower());
  }

  // モータに設定するPower値の上限の制限が行われているか確認するテスト
  TEST(WheelMotorControllerTest, SetLeftMotorPowerMax)
  {
    WheelMotorController wheelController;
    wheelController.setLeftPower(150);
    EXPECT_EQ(MOTOR_POWER_MAX, wheelController.getLeftPower());
  }

  // stopBoth()を呼び出して両モータのPower値が0になるかのテスト
  TEST(WheelMotorControllerTest, StopBoth)
  {
    WheelMotorController wheelController;
    wheelController.setLeftPower(50);
    wheelController.setRightPower(50);
    EXPECT_EQ(50, wheelController.getLeftPower());
    EXPECT_EQ(50, wheelController.getRightPower());
    wheelController.stopBoth();
    EXPECT_EQ(0, wheelController.getRightPower());
    EXPECT_EQ(0, wheelController.getLeftPower());
  }

  // brakeBoth()を呼び出して両モータのPower値が0になるかのテスト
  TEST(WheelMotorControllerTest, BrakeBoth)
  {
    WheelMotorController wheelController;
    wheelController.setLeftPower(50);
    wheelController.setRightPower(50);
    EXPECT_EQ(50, wheelController.getLeftPower());
    EXPECT_EQ(50, wheelController.getRightPower());
    wheelController.brakeBoth();
    EXPECT_EQ(0, wheelController.getRightPower());
    EXPECT_EQ(0, wheelController.getLeftPower());
  }
  // resetBothPower()で両モータの状態を0にset()したの動作テスト
  TEST(WheelMotorControllerTest, ResetBoth)
  {
    WheelMotorController wheelController;
    wheelController.setLeftPower(50);
    wheelController.setRightPower(50);
    EXPECT_EQ(50, wheelController.getLeftPower());
    EXPECT_EQ(50, wheelController.getRightPower());
    wheelController.resetBothPower();
    EXPECT_EQ(0, wheelController.getRightPower());
    EXPECT_EQ(0, wheelController.getLeftPower());
  }
}  // namespace etrobocon2026_test
