
/**
 * @file ArmMotorControllerTest.cpp
 * @brief ArmMotorControllerクラスをテストする
 * @author sadomiya-sousi
 */

#include "gtest/gtest.h"
#include "ArmMotorController.h"

using namespace spikeapi;
namespace etrobocon2026_test {

  // アームモータにPower値をセットできるかどうか確認するテスト
  TEST(ArmMotorControllerTest, SetPower)
  {
    ArmMotorController armController;
    armController.setPower(50);
    EXPECT_EQ(50, armController.getPower());
  }

  // stopMotor()を呼び出してアームモータのPower値が0になるかのテスト
  TEST(ArmMotorControllerTest, StopMotor)
  {
    ArmMotorController armController;
    armController.setPower(50);
    EXPECT_EQ(50, armController.getPower());
    armController.stopMotor();
    EXPECT_EQ(0, armController.getPower());
  }

  // holdArmMotor()を呼び出してアームモータのPower値が維持されるかのテスト
  TEST(ArmMotorControllerTest, HoldArmMotor)
  {
    ArmMotorController armController;
    armController.setPower(50);
    EXPECT_EQ(50, armController.getPower());
    armController.holdMotor();
    EXPECT_EQ(50, armController.getPower());
  }

}  // namespace etrobocon2026_test