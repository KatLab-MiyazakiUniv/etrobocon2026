/**
 * @file MotorControllerTest.cpp
 * @brief MotorControllerクラスをテストする
 * @author sadomiya-sousi
 */

#include "MotorController.h"
#include <gtest/gtest.h>

using namespace spikeapi;

namespace etrobocon2026_test {

  // 右車輪のモータにPower値をセットできるかのテスト
  TEST(MotorControllerTest, SetRightMotorPower)
  {
    MotorController controller;
    controller.setRightMotorPower(50);
    EXPECT_EQ(50, controller.getRightMotorPower());
  }

  // 右車輪のモータにマイナスのPower値をセットできるかどうかのテスト
  TEST(MotorControllerTest, SetRightMotorMinusPower)
  {
    MotorController controller;
    controller.setRightMotorPower(-50);
    EXPECT_EQ(-50, controller.getRightMotorPower());
  }

  // 左車輪のモータにPower値をセットできるかのテスト
  TEST(MotorControllerTest, SetLeftMotorPower)
  {
    MotorController controller;
    controller.setLeftMotorPower(50);
    EXPECT_EQ(50, controller.getLeftMotorPower());
  }

  // 左車輪のモータにマイナスのPower値をセットできるかどうかのテスト
  TEST(MotorControllerTest, SetLeftMotorMinusPower)
  {
    MotorController controller;
    controller.setLeftMotorPower(-50);
    EXPECT_EQ(-50, controller.getLeftMotorPower());
  }

  // モータに設定するPower値の下限の制限が行われているか確認するテスト
  TEST(MotorControllerTest, SetLeftMotorPowerMin)
  {
    MotorController controller;
    controller.setLeftMotorPower(-150);
    EXPECT_EQ(MotorController::MOTOR_POWER_MIN, controller.getLeftMotorPower());
  }

  // モータに設定するPower値の上限の制限が行われているか確認するテスト
  TEST(MotorControllerTest, SetLeftMotorPowerMax)
  {
    MotorController controller;
    controller.setLeftMotorPower(150);
    EXPECT_EQ(MotorController::MOTOR_POWER_MAX, controller.getLeftMotorPower());
  }

  // stopWheelsMotor()を呼び出せるか確認するテスト
  TEST(MotorControllerTest, StopWheelsMotor)
  {
    MotorController controller;
    controller.stopWheelsMotor();
    EXPECT_EQ(0, controller.getRightMotorPower());
    EXPECT_EQ(0, controller.getLeftMotorPower());
  }

  // brakeWheelsMotor()を呼び出せるか確認するテスト
  TEST(MotorControllerTest, BrakeWheelsMotor)
  {
    MotorController controller;
    controller.brakeWheelsMotor();
    EXPECT_EQ(0, controller.getRightMotorPower());
    EXPECT_EQ(0, controller.getLeftMotorPower());
  }

  // アームモータにPower値を設定できるかどうか確認するテスト
  TEST(MotorControllerTest, SetArmMotorPower)
  {
    MotorController controller;
    controller.setArmMotorPower(50);
    EXPECT_EQ(50, controller.getArmMotorPower());
  }

  // stopArmMotor()を呼び出せるか確認するテスト
  TEST(MotorControllerTest, StopArmMotor)
  {
    MotorController controller;
    controller.stopArmMotor();
    EXPECT_EQ(0, controller.getArmMotorPower());
  }

  // holdArmMotor()を呼び出せるか確認するテスト
  TEST(MotorControllerTest, HoldArmMotor)
  {
    MotorController controller;
    controller.holdArmMotor();
    EXPECT_EQ(0, controller.getArmMotorPower());
  }

}  // namespace etrobocon2026_test