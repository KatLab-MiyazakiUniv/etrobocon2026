/**
 * @file RobotTest.cpp
 * @brief Robotクラスのテスト
 * @author takuchi17
 */

#include <gtest/gtest.h>
#include "Robot.h"

namespace etrobocon2026_test {

  // ゲッターで取得した WheelMotorController インスタンスが等しいか確認するテスト
  TEST(RobotTest, GetWheelMotorControllerInstanceReturnsReference)
  {
    Robot robot;
    WheelMotorController& wheelMotorRef1 = robot.getWheelMotorControllerInstance();
    WheelMotorController& wheelMotorRef2 = robot.getWheelMotorControllerInstance();

    EXPECT_EQ(&wheelMotorRef1, &wheelMotorRef2);
  }

  // ゲッターで取得した ArmMotorController インスタンスが等しいか確認するテスト
  TEST(RobotTest, GetArmMotorControllerInstanceReturnsReference)
  {
    Robot robot;
    ArmMotorController& armMotorRef1 = robot.getArmMotorControllerInstance();
    ArmMotorController& armMotorRef2 = robot.getArmMotorControllerInstance();

    EXPECT_EQ(&armMotorRef1, &armMotorRef2);
  }

  // ゲッターで取得した IMUController インスタンスが等しいか確認するテスト
  TEST(RobotTest, GetIMUControllerInstanceReturnsReference)
  {
    Robot robot;
    IMUController& imuRef1 = robot.getIMUControllerInstance();
    IMUController& imuRef2 = robot.getIMUControllerInstance();

    EXPECT_EQ(&imuRef1, &imuRef2);
  }

}  // namespace etrobocon2026_test