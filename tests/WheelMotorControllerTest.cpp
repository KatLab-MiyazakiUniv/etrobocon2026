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
  TEST(WheelMotorControllerTest, SetRightMotorPower)
  {
    WheelMotorController wheelController;
    wheelController.setRightMotorPower(50);
    EXPECT_EQ(50, wheelController.getRightMotorPower());
  }

  // 右モータにマイナスのPower値をセットできるかどうかのテスト
  TEST(WheelMotorControllerTest, SetRightMotorMinusPower)
  {
    WheelMotorController wheelController;
    wheelController.setRightMotorPower(-50);
    EXPECT_EQ(-50, wheelController.getRightMotorPower());
  }

  // 左モータにPower値をセットできるかのテスト
  TEST(WheelMotorControllerTest, SetLeftMotorPower)
  {
    WheelMotorController wheelController;
    wheelController.setLeftMotorPower(50);
    EXPECT_EQ(50, wheelController.getLeftMotorPower());
  }

  // 左モータにマイナスのPower値をセットできるかどうかのテスト
  TEST(WheelMotorControllerTest, SetLeftMotorMinusPower)
  {
    WheelMotorController wheelController;
    wheelController.setLeftMotorPower(-50);
    EXPECT_EQ(-50, wheelController.getLeftMotorPower());
  }

  // モータに設定するPower値の下限の制限が行われているか確認するテスト
  TEST(WheelMotorControllerTest, SetLeftMotorPowerMin)
  {
    WheelMotorController wheelController;

    wheelController.setLeftMotorPower(-150);
    EXPECT_EQ(MOTOR_POWER_MIN, wheelController.getLeftMotorPower());
  }

  // モータに設定するPower値の上限の制限が行われているか確認するテスト
  TEST(WheelMotorControllerTest, SetLeftMotorPowerMax)
  {
    WheelMotorController wheelController;
    wheelController.setLeftMotorPower(150);
    EXPECT_EQ(MOTOR_POWER_MAX, wheelController.getLeftMotorPower());
  }

  // stopWheelsMotor()を呼び出せるか動作テスト
  TEST(WheelMotorControllerTest, StopWheelsMotor)
  {
    WheelMotorController wheelController;
    wheelController.setLeftMotorPower(50);
    wheelController.setRightMotorPower(50);
    EXPECT_EQ(50, wheelController.getLeftMotorPower());
    EXPECT_EQ(50, wheelController.getRightMotorPower());
    wheelController.stopWheelsMotor();
    EXPECT_EQ(0, wheelController.getRightMotorPower());
    EXPECT_EQ(0, wheelController.getLeftMotorPower());
  }

  // brakeWheelsMotor()を呼び出せるか動作テスト
  TEST(WheelMotorControllerTest, BrakeWheelsMotor)
  {
    WheelMotorController wheelController;
    wheelController.setLeftMotorPower(50);
    wheelController.setRightMotorPower(50);
    EXPECT_EQ(50, wheelController.getLeftMotorPower());
    EXPECT_EQ(50, wheelController.getRightMotorPower());
    wheelController.brakeWheelsMotor();
    EXPECT_EQ(0, wheelController.getRightMotorPower());
    EXPECT_EQ(0, wheelController.getLeftMotorPower());
  }
  // resetWheelsMotor()で両モータの状態を0にset()したの動作テスト
  TEST(WheelMotorControllerTest, ResetWheelsMotorPower)
  {
    WheelMotorController wheelController;
    wheelController.setLeftMotorPower(50);
    wheelController.setRightMotorPower(50);
    EXPECT_EQ(50, wheelController.getLeftMotorPower());
    EXPECT_EQ(50, wheelController.getRightMotorPower());
    wheelController.resetWheelsMotorPower();
    EXPECT_EQ(0, wheelController.getRightMotorPower());
    EXPECT_EQ(0, wheelController.getLeftMotorPower());
  }
}  // namespace etrobocon2026_test
