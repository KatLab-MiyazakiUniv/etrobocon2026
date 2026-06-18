/**
 * @file MotionTimeConditionTest.cpp
 * @brief MotionTimeConditionクラスをテストする
 * @author takahashitom
 */

#include <gtest/gtest.h>
#include "MotionTimeCondition.h"
#include "MockNetworkSystem.h"

namespace etrobocon2026_test {

  // 指定時間に達していない場合、shouldContinue() が true を返すかテスト
  TEST(MotionTimeConditionTest, ShouldContinueTrue)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    int targetTime = 10000;

    MotionTimeCondition condition(robot, targetTime);
    condition.prepare();

    EXPECT_TRUE(condition.shouldContinue());
  }

  // 指定時間に達した場合、shouldContinue() が false を返すかテスト
  TEST(MotionTimeConditionTest, ShouldContinueFalse)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    int targetTime = 100;

    MotionTimeCondition condition(robot, targetTime);
    condition.prepare();

    // 指定時間に達するまで待機
    ClockUtil::sleep(100);

    EXPECT_FALSE(condition.shouldContinue());
  }

  // 指定時間が 0 の場合、shouldContinue() がすぐに false を返すかテスト
  TEST(MotionTimeConditionTest, ZeroTargetTime)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    int targetTime = 0;

    MotionTimeCondition condition(robot, targetTime);
    condition.prepare();

    EXPECT_FALSE(condition.shouldContinue());
  }

  // 指定時間が負の場合、shouldContinue() がすぐに false を返すかテスト
  TEST(MotionTimeConditionTest, MinusTargetTime)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    int targetTime = -10;

    MotionTimeCondition condition(robot, targetTime);
    condition.prepare();

    EXPECT_FALSE(condition.shouldContinue());
  }

}  // namespace etrobocon2026_test
