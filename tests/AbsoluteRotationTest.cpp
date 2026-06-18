/**
 * @file AbsoluteRotationTest.cpp
 * @brief AbsoluteRotationクラスをテストする
 * @author okuyama0528
 */

#include <gtest/gtest.h>
#include "AbsoluteRotation.h"
#include "AbsoluteAngleCondition.h"
#include "MockNetworkSystem.h"

namespace etrobocon2026_test {
  class AbsoluteRotationTest : public ::testing::Test {
   protected:
    // 回転用PIDゲイン
    Pid::PidGain anglePid = { 0.005, 0.001, 0.0 };
  };

  // run()で回転後、角度が目標角度だけ増加するかテスト（誤差あり）
  TEST_F(AbsoluteRotationTest, RunAngle)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double AbsoluteAngle = 60.0;  //  回頭したい相対角度

    double initialAngle = robot.getIMUControllerInstance().getAzimuth();  // 回頭前の角度を計算

    // 回転動作を実行
    AbsoluteRotation AbsoluteRotation(
        robot, std::make_unique<AbsoluteAngleCondition>(robot, AbsoluteAngle), anglePid,
        AbsoluteAngle);
    AbsoluteRotation.run();

    // 回転後の角度を計算
    double endAngle = robot.getIMUControllerInstance().getAzimuth();

    // 角度と目標角度との誤差
    double deviation = 2.0;

    // 回転後の角度が目標角度の誤差の範囲にあることテスト
    EXPECT_NEAR(AbsoluteAngle, endAngle - initialAngle, deviation);
  }

  // run()で回転後、右タイヤのpowor値が0かのテスト
  TEST_F(AbsoluteRotationTest, RunRightPower)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double AbsoluteAngle = 45.0;  //  回頭したい相対角度

    // 回転動作を実行
    AbsoluteRotation AbsoluteRotation(
        robot, std::make_unique<AbsoluteAngleCondition>(robot, AbsoluteAngle), anglePid,
        AbsoluteAngle);
    AbsoluteRotation.run();

    // 回転後の右タイヤのpower値を取得
    double rightPower = robot.getWheelMotorControllerInstance().getRightPower();

    EXPECT_EQ(0, rightPower);
  }

  // run()で回転後、左タイヤのpowor値が0かのテスト
  TEST_F(AbsoluteRotationTest, RunLeftPower)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double AbsoluteAngle = 45.0;  //  回頭したい相対角度

    // 回転動作を実行
    AbsoluteRotation AbsoluteRotation(
        robot, std::make_unique<AbsoluteAngleCondition>(robot, AbsoluteAngle), anglePid,
        AbsoluteAngle);
    AbsoluteRotation.run();

    // 回転後の左タイヤのpower値を取得
    double leftPower = robot.getWheelMotorControllerInstance().getLeftPower();

    EXPECT_EQ(0, leftPower);
  }
}  // namespace etrobocon2026_test
