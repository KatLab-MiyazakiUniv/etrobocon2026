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

  // 目標角度が正の時、run()で回転後、角度が目標角度だけ増加するかテスト（誤差あり）
  TEST_F(RelativeRotationTest, Run)
  {
    Robot robot;
    double relativeAngle = 60;  //  回頭したい相対角度

    double initialAngle = robot.getIMUControllerInstance().getAzimuth();  // 回頭前の角度を計算

    // 回転動作を実行
    RelativeRotation relativeRotation(
        robot, std::make_unique<RelativeAngleContinuationCondition>(robot, relativeAngle), anglePid,
        relativeAngle);
    relativeRotation.run();

    // 回転後の角度を計算
    double endAngle = robot.getIMUControllerInstance().getAzimuth();

    // 角度と目標角度との誤差
    double deviation = 90;

    // 回転後の角度が目標角度の誤差の範囲にあることテスト
    EXPECT_NEAR(relativeAngle, endAngle - initialAngle, deviation);
  }

  // 目標角度が負の時、run()で回転後、角度が目標角度だけ増加するかテスト（誤差あり）
  TEST_F(RelativeRotationTest, RunNegative)
  {
    Robot robot;
    double relativeAngle = -45.0;  //  回頭したい相対角度

    double initialAngle = robot.getIMUControllerInstance().getAzimuth();  // 回頭前の角度を計算

    // 回転動作を実行
    RelativeRotation relativeRotation(
        robot, std::make_unique<RelativeAngleContinuationCondition>(robot, relativeAngle), anglePid,
        relativeAngle);
    relativeRotation.run();

    // 回転後の角度を計算
    double endAngle = robot.getIMUControllerInstance().getAzimuth();

    // 角度と目標角度との誤差
    double deviation = 2.0;

    // 回転後の角度が目標角度の誤差の範囲にあることテスト
    EXPECT_NEAR(relativeAngle, endAngle - initialAngle, deviation);
  }
  // 目標角度が0の時、run()で回頭後、角度が増加しないことをテスト（誤差あり）
  TEST_F(RelativeRotationTest, RunZero)
  {
    Robot robot;
    double relativeAngle = 0.0;  //  回頭したい相対角度

    double initialAngle = robot.getIMUControllerInstance().getAzimuth();  // 回頭前の角度を計算

    // 回転動作を実行
    RelativeRotation relativeRotation(
        robot, std::make_unique<RelativeAngleContinuationCondition>(robot, relativeAngle), anglePid,
        relativeAngle);
    relativeRotation.run();

    // 回転後の角度を計算
    double endAngle = robot.getIMUControllerInstance().getAzimuth();

    // 角度と目標角度との誤差
    double deviation = 80.0;

    // 回転後の角度が目標角度の誤差の範囲にあることテスト
    EXPECT_NEAR(relativeAngle, endAngle - initialAngle, deviation);
  }
  // 目標角度が360度より大きい時、run()で回転後、角度が目標角度だけ増加するかテスト（誤差あり）
  TEST_F(RelativeRotationTest, RunOver360)
  {
    Robot robot;
    double relativeAngle = 380;  //  回頭したい相対角度

    double initialAngle = robot.getIMUControllerInstance().getAzimuth();  // 回頭前の角度を計算
    relativeAngle = fmod(relativeAngle + 180.0, 360.0);
    if(relativeAngle < 0) relativeAngle += 360.0;
    // 回転動作を実行
    RelativeRotation relativeRotation(
        robot, std::make_unique<RelativeAngleContinuationCondition>(robot, relativeAngle), anglePid,
        relativeAngle);
    relativeRotation.run();

    // 回転後の角度を計算
    double endAngle = robot.getIMUControllerInstance().getAzimuth();

    // 角度と目標角度との誤差
    double deviation = 80.0;

    // 回転後の角度が目標角度の誤差の範囲にあることテスト
    EXPECT_NEAR(relativeAngle, endAngle - initialAngle, deviation);
  }
  // 目標角度が-360度より大きい時、run()で回転後、角度が目標角度だけ増加するかテスト（誤差あり）
  TEST_F(RelativeRotationTest, RunOverNegative360)
  {
    Robot robot;
    double relativeAngle = -4000;  //  回頭したい相対角度
    relativeAngle = fmod(relativeAngle + 180.0, 360.0);
    if(relativeAngle < 0) relativeAngle += 360.0;

    double initialAngle = robot.getIMUControllerInstance().getAzimuth();  // 回頭前の角度を計算

    // 回転動作を実行
    RelativeRotation relativeRotation(
        robot, std::make_unique<RelativeAngleContinuationCondition>(robot, relativeAngle), anglePid,
        relativeAngle);
    relativeRotation.run();

    // 回転後の角度を計算
    double endAngle = robot.getIMUControllerInstance().getAzimuth();

    // 角度と目標角度との誤差
    double deviation = 80.0;

    // 回転後の角度が目標角度の誤差の範囲にあることテスト
    EXPECT_NEAR(relativeAngle, endAngle - initialAngle, deviation);
  }
}  // namespace etrobocon2026_test
