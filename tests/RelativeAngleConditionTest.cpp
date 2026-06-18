/**
 * @file RelativeAngleConditionTest.cpp
 * @brief RelativeAngleConditionクラスをテストする
 * @author okuyama0528 yutaro-1214
 */

#include <gtest/gtest.h>
#include "RelativeAngleCondition.h"
#include "AngleNormalizer.h"
#include "MockNetworkSystem.h"

namespace etrobocon2026_test {
  // prepare()実行後に、targetAngleが正しくセット出来ているか確認
  TEST(RelativeAngleConditionTest, AfterPrepare)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetAngle = 100.0;
    RelativeAngleCondition condition(robot, targetAngle);

    double expectedTargetAngle = AngleNormalizer::normalizeAngle(targetAngle);

    condition.prepare();

    EXPECT_EQ(expectedTargetAngle, condition.getTargetAngle());
  }

  // 目標角度に到達していない場合は継続判定になることを確認
  TEST(RelativeAngleConditionConditionTest, NotReachTargetAngle)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    RelativeAngleCondition condition(robot, 100.0);

    condition.prepare();

    EXPECT_TRUE(condition.shouldContinue());
  }

  // 角度が目標角度に到達した場合は停止判定になることを確認
  TEST(RelativeAngleConditionConditionTest, AfterTargetAngle)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetAngle = 20;
    // initialAngleとconditionで2回getAzimuthを実行するため初期値は2
    int i = 2;

    double initialAngle = robot.getIMUControllerInstance().getAzimuth();

    RelativeAngleCondition condition(robot, targetAngle);

    condition.prepare();

    // 目標角度に到達するまで回頭
    while(condition.shouldContinue()) {
      i++;
    }

    // 目標角度付近まで回っていることを確認
    EXPECT_NEAR(targetAngle, i, 2);
  }

  /**
   * 目標角度が0の場合は即停止判定になることを確認
   */
  TEST(RelativeAngleConditionConditionTest, ZeroTargetAbgle)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    RelativeAngleCondition condition(robot, 0.0);

    condition.prepare();

    EXPECT_FALSE(condition.shouldContinue());
  }

}  // namespace etrobocon2026_test