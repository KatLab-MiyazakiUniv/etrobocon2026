/**
 * @file   GoalNavigationTest.cpp
 * @brief  GoalNavigationクラスをテストする
 * @author yutaro-1214
 */

#include <cmath>
#include <gtest/gtest.h>
#include <memory>

#include "GoalNavigation.h"
#include "MockNetworkSystem.h"
#include "RepeatCountCondition.h"

namespace etrobocon2026_test {

  class GoalNavigationTest : public ::testing::Test {
   protected:
    // 回転用PIDゲイン
    Pid::PidGain rotationPid = { 0.005, 0.001, 0.0 };

    // 右タイヤ速度制御用PIDゲイン
    Pid::PidGain rightPid = { 0.016, 0.005, 0.0015 };

    // 左タイヤ速度制御用PIDゲイン
    Pid::PidGain leftPid = { 0.016, 0.0045, 0.0015 };

    // 直進時の角度制御用PIDゲイン
    Pid::PidGain straightAnglePid = { 0.02, 0.006, 0.003 };

    // 直進速度[mm/s]
    double targetSpeed = 300.0;
  };

  // run()実行後、ロボットのX座標が目標X座標付近になるかのテスト
  TEST_F(GoalNavigationTest, RunTargetX)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetX = 100.0;
    double targetY = 0.0;

    GoalNavigation goalNavigation(robot, std::make_unique<RepeatCountCondition>(robot, 1), targetX,
                                  targetY, targetSpeed, rotationPid, rightPid, leftPid,
                                  straightAnglePid);

    goalNavigation.run();

    double actualX = robot.getPositionInstance().getX();

    // DistanceConditionやエンコーダの誤差を考慮する
    double deviation = 15.0;

    EXPECT_NEAR(targetX, actualX, deviation);
  }

  // run()実行後、ロボットのY座標が目標Y座標付近になるかのテスト
  TEST_F(GoalNavigationTest, RunTargetY)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetX = 0.0;
    double targetY = 100.0;

    GoalNavigation goalNavigation(robot, std::make_unique<RepeatCountCondition>(robot, 1), targetX,
                                  targetY, targetSpeed, rotationPid, rightPid, leftPid,
                                  straightAnglePid);

    goalNavigation.run();

    double actualY = robot.getPositionInstance().getY();

    double deviation = 15.0;

    EXPECT_NEAR(targetY, actualY, deviation);
  }

  // run()実行後、ロボットが目標地点付近に到達するかのテスト
  TEST_F(GoalNavigationTest, RunTargetPosition)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetX = 100.0;
    double targetY = 100.0;

    GoalNavigation goalNavigation(robot, std::make_unique<RepeatCountCondition>(robot, 1), targetX,
                                  targetY, targetSpeed, rotationPid, rightPid, leftPid,
                                  straightAnglePid);

    goalNavigation.run();

    double actualX = robot.getPositionInstance().getX();
    double actualY = robot.getPositionInstance().getY();

    double dx = targetX - actualX;
    double dy = targetY - actualY;

    // 実行後の位置から目標地点までの残り距離
    double remainingDistance = std::sqrt(dx * dx + dy * dy);

    // GoalNavigationの到達許容距離と同じ10mm程度を基準にする
    double deviation = 15.0;

    EXPECT_LE(remainingDistance, deviation);
  }

  // run()実行後、ロボットの方位角が目標地点への方位角付近になるかのテスト
  TEST_F(GoalNavigationTest, RunTargetHeading)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetX = 100.0;
    double targetY = 100.0;

    double expectedHeading = 45.0;

    GoalNavigation goalNavigation(robot, std::make_unique<RepeatCountCondition>(robot, 1), targetX,
                                  targetY, targetSpeed, rotationPid, rightPid, leftPid,
                                  straightAnglePid);

    goalNavigation.run();

    double actualHeading = robot.getIMUControllerInstance().getAzimuth();

    // AbsoluteRotationの許容誤差に合わせる
    double deviation = 2.0;

    EXPECT_NEAR(expectedHeading, actualHeading, deviation);
  }

  // 目標地点がロボットの後方にある場合、
  // 回頭後に目標地点付近まで移動できるかのテスト
  TEST_F(GoalNavigationTest, RunTargetBehind)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetX = -100.0;
    double targetY = 0.0;

    GoalNavigation goalNavigation(robot, std::make_unique<RepeatCountCondition>(robot, 1), targetX,
                                  targetY, targetSpeed, rotationPid, rightPid, leftPid,
                                  straightAnglePid);

    goalNavigation.run();

    double actualX = robot.getPositionInstance().getX();
    double actualY = robot.getPositionInstance().getY();

    double dx = targetX - actualX;
    double dy = targetY - actualY;
    double remainingDistance = std::sqrt(dx * dx + dy * dy);

    double deviation = 15.0;

    EXPECT_LE(remainingDistance, deviation);
  }

  // すでに目標地点にいる場合、位置が変化しないかのテスト
  TEST_F(GoalNavigationTest, RunAlreadyAtTarget)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetX = 0.0;
    double targetY = 0.0;

    GoalNavigation goalNavigation(robot, std::make_unique<RepeatCountCondition>(robot, 1), targetX,
                                  targetY, targetSpeed, rotationPid, rightPid, leftPid,
                                  straightAnglePid);

    goalNavigation.run();

    double actualX = robot.getPositionInstance().getX();
    double actualY = robot.getPositionInstance().getY();

    EXPECT_DOUBLE_EQ(0.0, actualX);
    EXPECT_DOUBLE_EQ(0.0, actualY);
  }

  // run()終了後、右タイヤのpower値が0かのテスト
  TEST_F(GoalNavigationTest, RunRightPower)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetX = 100.0;
    double targetY = 0.0;

    GoalNavigation goalNavigation(robot, std::make_unique<RepeatCountCondition>(robot, 1), targetX,
                                  targetY, targetSpeed, rotationPid, rightPid, leftPid,
                                  straightAnglePid);

    goalNavigation.run();

    double rightPower = robot.getWheelMotorControllerInstance().getRightPower();

    EXPECT_EQ(0, rightPower);
  }

  // run()終了後、左タイヤのpower値が0かのテスト
  TEST_F(GoalNavigationTest, RunLeftPower)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetX = 100.0;
    double targetY = 0.0;

    GoalNavigation goalNavigation(robot, std::make_unique<RepeatCountCondition>(robot, 1), targetX,
                                  targetY, targetSpeed, rotationPid, rightPid, leftPid,
                                  straightAnglePid);

    goalNavigation.run();

    double leftPower = robot.getWheelMotorControllerInstance().getLeftPower();

    EXPECT_EQ(0, leftPower);
  }

}  // namespace etrobocon2026_test