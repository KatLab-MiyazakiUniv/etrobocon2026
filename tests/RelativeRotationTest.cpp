/**
 * @file RelativeRotationTest.cpp
 * @brief RelativeRotationクラスをテストする
 * @author okuyama0528
 */

#include <gtest/gtest.h>
#include "Rotation.h"
#include "RelativeRotation.h"
#include "AngleCondition.h"
#include "RelativeAngleCondition.h"
#include "MockNetworkSystem.h"

namespace etrobocon2026_test {
  class RelativeRotationTest : public ::testing::Test {
   protected:
    // 回頭用PIDゲイン
    Pid::PidGain anglePid = { 0.005, 0.001, 0.0 };
  };

  // run()回頭後、角度が目標角度だけ増加するかテスト（誤差あり）
  TEST_F(RelativeRotationTest, RunAngle)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double relativeAngle = 60.0;  //  回頭したい相対角度

    double initialAngle = robot.getIMUControllerInstance().getAzimuth();  // 回頭前の角度を計算

    // 回頭動作を実行
    RelativeRotation relativeRotation(
        robot, std::make_unique<RelativeAngleCondition>(robot, relativeAngle), anglePid,
        relativeAngle);
    relativeRotation.run();

    // 回頭後の角度を計算
    double endAngle = robot.getIMUControllerInstance().getAzimuth();

    // 角度と目標角度との誤差
    double deviation = 2.0;

    // 回頭後の角度が目標角度の誤差の範囲にあることテスト
    EXPECT_NEAR(relativeAngle, endAngle - initialAngle, deviation);
  }

  // run()で回頭後、右タイヤのpowor値が0かのテスト
  TEST_F(RelativeRotationTest, RunRightPower)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double relativeAngle = 45.0;  //  回頭したい相対角度

    // 回頭動作を実行
    RelativeRotation relativeRotation(
        robot, std::make_unique<RelativeAngleCondition>(robot, relativeAngle), anglePid,
        relativeAngle);
    relativeRotation.run();

    // 回頭後の右タイヤのpower値を取得
    double rightPower = robot.getWheelMotorControllerInstance().getRightPower();

    EXPECT_EQ(0, rightPower);
  }

  // run()で回頭後、左タイヤのpowor値が0かのテスト
  TEST_F(RelativeRotationTest, RunLeftPower)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    double relativeAngle = 45.0;  //  回頭したい相対角度

    // 回頭動作を実行
    RelativeRotation relativeRotation(
        robot, std::make_unique<RelativeAngleCondition>(robot, relativeAngle), anglePid,
        relativeAngle);
    relativeRotation.run();

    // 回頭後の左タイヤのpower値を取得
    double leftPower = robot.getWheelMotorControllerInstance().getLeftPower();

    EXPECT_EQ(0, leftPower);
  }
}  // namespace etrobocon2026_test
