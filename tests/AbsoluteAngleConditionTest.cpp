/**
 * @file AbsoluteAngleConditionTest.cpp
 * @brief AbsoluteAngleConditionクラスをテストする
 * @author okuyama0528 yutaro-1214
 */

#include <gtest/gtest.h>
#include "AbsoluteAngleCondition.h"
#include "MockNetworkSystem.h"

namespace etrobocon2026_test {
  // prepare()実行後に、targetAngleが正しくセット出来ているか確認
  TEST(AbsoluteAngleConditionTest, AfterPrepare)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetAngle = 100.0;
    AbsoluteAngleCondition condition(robot, targetAngle);

    double expectedTargetAngle = targetAngle;

    condition.prepare();

    EXPECT_EQ(expectedTargetAngle, condition.getTargetAngle());
  }

  // 目標角度に到達していない場合は継続判定になることを確認
  TEST(AbsoluteAngleConditionConditionTest, NotReachTargetAngle)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    AbsoluteAngleCondition condition(robot, 100.0);

    condition.prepare();

    EXPECT_TRUE(condition.shouldContinue());
  }

  // 角度が目標角度に到達した場合は停止判定になることを確認
  TEST(AbsoluteAngleConditionConditionTest, AfterTargetAngle)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    int i = 0;
    double targetAngle = 20;

    AbsoluteAngleCondition condition(robot, targetAngle);

    condition.prepare();

    // 目標角度に到達するまで回頭
    while(condition.shouldContinue()) {
      i++;
    }

    // 目標角度付近まで回っていることを確認
    EXPECT_NEAR(targetAngle, robot.getIMUControllerInstance().getAzimuth(), 2);
  }

}  // namespace etrobocon2026_test