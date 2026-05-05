
/**
 * @file ArmMotorControllerTest.cpp
 * @brief ArmMotorControllerクラスをテストする
 * @author sadomiya-sousi
 */

#include "gtest/gtest.h"
#include "ArmMotorController.h"

using namespace spikeapi;
namespace etrobocon2026_test {

  // アームモータにPower値を設定できるかどうか確認するテスト
  TEST(ArmMotorControllerTest, SetArmMotorPower)
  {
    ArmMotorController armController;
    armController.setArmMotorPower(50);
    EXPECT_EQ(50, armController.getArmMotorPower());
  }

  // stopArmMotor()を呼び出せるか確認するテスト
  TEST(ArmMotorControllerTest, StopArmMotor)
  {
    ArmMotorController armController;
    armController.stopArmMotor();
    EXPECT_EQ(0, armController.getArmMotorPower());
  }

  // holdArmMotor()を呼び出せるか確認するテスト
  TEST(ArmMotorControllerTest, HoldArmMotor)
  {
    ArmMotorController armController;
    armController.holdArmMotor();
    EXPECT_EQ(0, armController.getArmMotorPower());
  }

}  // namespace etrobocon2026_test