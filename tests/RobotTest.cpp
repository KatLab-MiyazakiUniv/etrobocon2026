/**
 * @file RobotTest.cpp
 * @brief Robotクラスのテスト
 * @author takuchi17
 */

#include <gtest/gtest.h>
#include "Robot.h"
#include "MockNetworkSystem.h"

namespace etrobocon2026_test {

  // ゲッターで取得した WheelMotorController インスタンスが等しいか確認するテスト
  TEST(RobotTest, GetWheelMotorControllerInstanceReturnsReference)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    WheelMotorController& wheelMotorRef1 = robot.getWheelMotorControllerInstance();
    WheelMotorController& wheelMotorRef2 = robot.getWheelMotorControllerInstance();

    EXPECT_EQ(&wheelMotorRef1, &wheelMotorRef2);
  }

  // ゲッターで取得した ArmMotorController インスタンスが等しいか確認するテスト
  TEST(RobotTest, GetArmMotorControllerInstanceReturnsReference)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    ArmMotorController& armMotorRef1 = robot.getArmMotorControllerInstance();
    ArmMotorController& armMotorRef2 = robot.getArmMotorControllerInstance();

    EXPECT_EQ(&armMotorRef1, &armMotorRef2);
  }

  // ゲッターで取得した IMUController インスタンスが等しいか確認するテスト
  TEST(RobotTest, GetIMUControllerInstanceReturnsReference)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    IMUController& imuRef1 = robot.getIMUControllerInstance();
    IMUController& imuRef2 = robot.getIMUControllerInstance();

    EXPECT_EQ(&imuRef1, &imuRef2);
  }

}  // namespace etrobocon2026_test