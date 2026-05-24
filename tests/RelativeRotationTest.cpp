/**
 * @file RelativeRotationTest.cpp
 * @brief RelativeRotationクラスをテストする
 * @author okuyama0528
 */

#include <gtest/gtest.h>
#include "Rotation.h"
#include "RelativeRotation.h"
#include "AngleContinuationCondition.h"
#include "RelativeAngleContinuationCondition.h"

namespace etrobocon2026_test {
  class RelativeRotationTest : public ::testing::Test {
   protected:
    // 回転用PIDゲイン
    Pid::PidGain anglePid = { 0.005, 0.001, 0.0 };
  };

  // run()で回転後、角度が目標角度だけ増加するかテスト（誤差あり）
  TEST_F(RelativeRotationTest, RunAngle)
  {
    Robot robot;
    double relativeAngle = 60;  //  回頭したい相対角度

    double initialAngle = robot.getIMUControllerInstance().getAzimuth();  // 回頭前の角度を計算

    // 回転動作を実行
    RelativeRotation relativeRotation(
        robot, std::make_unique<RelativeAngleContinuationCondition>(robot, relativeAngle), anglePid,
        relativeAngle, 0.0);
    relativeRotation.run();

    // 回転後の角度を計算
    double endAngle = robot.getIMUControllerInstance().getAzimuth();

    // 角度と目標角度との誤差
    double deviation = 2;

    // 回転後の角度が目標角度の誤差の範囲にあることテスト
    EXPECT_NEAR(relativeAngle, endAngle - initialAngle, deviation);
  }

  // run()で回転後、右タイヤのpowor値が0かのテスト
  TEST_F(RelativeRotationTest, RunRightPower)
  {
    Robot robot;
    double relativeAngle = 45.0;  //  回頭したい相対角度

    // 回転動作を実行
    RelativeRotation relativeRotation(
        robot, std::make_unique<RelativeAngleContinuationCondition>(robot, relativeAngle), anglePid,
        relativeAngle, 0.0);
    relativeRotation.run();

    // 回転後の右タイヤのpower値を取得
    double rightPower = robot.getWheelMotorControllerInstance().getRightPower();

    EXPECT_EQ(0, rightPower);
  }

  // run()で回転後、左タイヤのpowor値が0かのテスト
  TEST_F(RelativeRotationTest, RunLeftPower)
  {
    Robot robot;
    double relativeAngle = 45.0;  //  回頭したい相対角度

    // 回転動作を実行
    RelativeRotation relativeRotation(
        robot, std::make_unique<RelativeAngleContinuationCondition>(robot, relativeAngle), anglePid,
        relativeAngle, 0.0);
    relativeRotation.run();

    // 回転後の左タイヤのpower値を取得
    double leftPower = robot.getWheelMotorControllerInstance().getLeftPower();

    EXPECT_EQ(0, leftPower);
  }
}  // namespace etrobocon2026_test
