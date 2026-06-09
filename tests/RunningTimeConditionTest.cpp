/**
 * @file RunningTimeConditionTest.cpp
 * @brief RunningTimeConditionクラスをテストする
 * @author takahashitom
 */

#include <gtest/gtest.h>
#include "RunningTimeCondition.h"

namespace etrobocon2026_test {

  // 指定時間に達していない場合、shouldContinue() が true を返すかテスト
  TEST(RunningTimeConditionTest, ShouldContinueTrue)
  {
    Robot robot;

    // 走行開始時間をセット
    robot.setRunningStartTime(ClockUtil::now());

    int targetTime = 10000;

    RunningTimeCondition condition(robot, targetTime);

    EXPECT_TRUE(condition.shouldContinue());
  }

  // 指定時間に達した場合、shouldContinue() が false を返すかテスト
  TEST(RunningTimeConditionTest, ShouldContinueFalse)
  {
    Robot robot;

    // 走行開始時間をセット
    robot.setRunningStartTime(ClockUtil::now());

    int targetTime = 100;

    RunningTimeCondition condition(robot, targetTime);

    // 指定時間にまだ達していないことを確認
    EXPECT_TRUE(condition.shouldContinue());

    // 指定時間に達するまで待機
    ClockUtil::sleep(100);

    EXPECT_FALSE(condition.shouldContinue());
  }

  // 指定時間が既に経過していた場合、shouldContinue() がすぐに false を返すかテスト
  TEST(RunningTimeConditionTest, AlreadyTargetTime)
  {
    Robot robot;

    // 走行開始時間をセット
    robot.setRunningStartTime(ClockUtil::now());

    ClockUtil::sleep(500);

    int targetTime = 100;

    RunningTimeCondition condition(robot, targetTime);

    EXPECT_FALSE(condition.shouldContinue());
  }

  // 指定時間が 0 の場合、shouldContinue() がすぐに false を返すかテスト
  TEST(RunningTimeConditionTest, ZeroTargetTime)
  {
    Robot robot;

    // 走行開始時間をセット
    robot.setRunningStartTime(ClockUtil::now());

    int targetTime = 0;

    RunningTimeCondition condition(robot, targetTime);

    EXPECT_FALSE(condition.shouldContinue());
  }

  // 指定時間が負の場合、shouldContinue() がすぐに false を返すかテスト
  TEST(RunningTimeConditionTest, MinusTargetTime)
  {
    Robot robot;

    // 走行開始時間をセット
    robot.setRunningStartTime(ClockUtil::now());

    int targetTime = -10;

    RunningTimeCondition condition(robot, targetTime);

    EXPECT_FALSE(condition.shouldContinue());
  }

}  // namespace etrobocon2026_test
