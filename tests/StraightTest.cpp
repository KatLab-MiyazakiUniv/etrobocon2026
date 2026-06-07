/**
 * @file StraightTest.cpp
 * @brief Straightクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "Straight.h"
#include "Mileage.h"
#include "DistanceCondition.h"
#include "MockNetworkSystem.h"
#include <iostream>

namespace etrobocon2026_test {
  class StraightTest : public ::testing::Test {
   protected:
    // テスト用のPidゲイン
    Pid::PidGain rightPid = { 0.00535, 0.00115, 0.000 };
    Pid::PidGain leftPid = { 0.00578, 0.0008535, 0.000 };
    Pid::PidGain anglePid = { 0.036, 0.012, 0.03 };
  };

  // IMU無しの場合、目標距離が正の時、run()で直進後、走行距離が目標距離だけ増加するかテスト（誤差あり）
  TEST_F(StraightTest, Run)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double targetSpeed = 1000.0;   // 目標速度
    double targetDistance = 10.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed, rightPid, leftPid, anglePid, false);
    straight.run();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 走行距離と目標距離との誤差
    double deviation = 0.5;

    // 走行距離が目標距離の誤差の範囲にあることテスト
    EXPECT_NEAR(targetDistance, endMileage - startMileage, deviation);
  }

  // IMU無しの場合、目標距離が0の時、runで直進後、走行距離が増加しないことをテスト
  TEST_F(StraightTest, RunWithZeroDistance)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double targetSpeed = 50.0;    // 目標速度
    double targetDistance = 0.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed, rightPid, leftPid, anglePid, false);
    straight.run();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 目標距離と走行距離が等しいことをテスト
    EXPECT_EQ(targetDistance, endMileage - startMileage);
  }

  // IMU無しの場合、目標距離が負の時、runで直進後、走行距離が増加しないことをテスト
  TEST_F(StraightTest, RunWithNegativeDistance)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double targetSpeed = 50.0;       // 目標速度
    double targetDistance = -100.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed, rightPid, leftPid, anglePid, false);
    straight.run();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 走行距離が0であることをテスト
    double expected = 0.0;
    EXPECT_EQ(expected, endMileage - startMileage);
  }

  // IMU無しの場合、目標距離が正、目標速度が0の場合、走行距離0でrun()が終了するか確認
  TEST_F(StraightTest, RunWithZeroSpeed)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double targetSpeed = 0.0;      // 目標速度
    double targetDistance = 10.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed, rightPid, leftPid, anglePid, false);
    straight.run();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 走行距離が0であることをテスト
    double expected = 0.0;
    EXPECT_EQ(expected, endMileage - startMileage);
  }

  // IMU有りの場合、目標距離が正の時、run()で直進後、走行距離が目標距離だけ増加するかテスト（誤差あり）
  TEST_F(StraightTest, ImuRun)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double targetSpeed = 1000.0;   // 目標速度
    double targetDistance = 10.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed, rightPid, leftPid, anglePid, true);
    straight.run();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 走行距離と目標距離との誤差
    double deviation = 0.5;

    // 走行距離が目標距離の誤差の範囲にあることテスト
    EXPECT_NEAR(targetDistance, endMileage - startMileage, deviation);
  }

  // IMU有りの場合、目標距離が0の時、runで直進後、走行距離が増加しないことをテスト
  TEST_F(StraightTest, ImuRunZeroDistance)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double targetSpeed = 50.0;    // 目標速度
    double targetDistance = 0.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed, rightPid, leftPid, anglePid, true);
    straight.run();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 目標距離と走行距離が等しいことをテスト
    EXPECT_EQ(targetDistance, endMileage - startMileage);
  }

  // IMU有りの場合、目標距離が負の時、runで直進後、走行距離が増加しないことをテスト
  TEST_F(StraightTest, ImuRunNegativeDistance)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double targetSpeed = 50.0;       // 目標速度
    double targetDistance = -100.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed, rightPid, leftPid, anglePid, true);
    straight.run();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 走行距離が0であることをテスト
    double expected = 0.0;
    EXPECT_EQ(expected, endMileage - startMileage);
  }

  // IMU有りの場合、目標距離が正、目標速度が0の場合、走行距離0でrun()が終了するか確認
  TEST_F(StraightTest, ImuRunWithZeroSpeed)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double targetSpeed = 0.0;      // 目標速度
    double targetDistance = 10.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed, rightPid, leftPid, anglePid, true);
    straight.run();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 走行距離が0であることをテスト
    double expected = 0.0;
    EXPECT_EQ(expected, endMileage - startMileage);
  }

}  // namespace etrobocon2026_test
