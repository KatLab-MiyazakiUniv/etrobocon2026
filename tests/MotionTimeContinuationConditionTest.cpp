/**
 * @file MotionTimeContinuationConditionTest.cpp
 * @brief MotionTimeContinuationConditionクラスをテストする
 * @author takahashitom
 */

#include <gtest/gtest.h>
#include "MotionTimeContinuationCondition.h"

namespace etrobocon2026_test {

  // 指定時間に達していない場合、shouldContinue() が true を返すかテスト
  TEST(MotionTimeContinuationConditionTest, ShouldContinueTrue)
  {
    Robot robot;
    int targetTime = 10000;

    MotionTimeContinuationCondition condition(robot, targetTime);
    condition.prepare();

    EXPECT_TRUE(condition.shouldContinue());
  }

  // 指定時間に達した場合、shouldContinue() が false を返すかテスト
  TEST(MotionTimeContinuationConditionTest, ShouldContinueFalse)
  {
    Robot robot;
    int targetTime = 100;

    MotionTimeContinuationCondition condition(robot, targetTime);
    condition.prepare();

    // 指定時間に達するまで待機
    ClockUtil::sleep(100);

    EXPECT_FALSE(condition.shouldContinue());
  }

  // 指定時間が 0 の場合、shouldContinue() がすぐに false を返すかテスト
  TEST(MotionTimeContinuationConditionTest, ZeroTargetTime)
  {
    Robot robot;
    int targetTime = 0;

    MotionTimeContinuationCondition condition(robot, targetTime);
    condition.prepare();

    EXPECT_FALSE(condition.shouldContinue());
  }

  // 指定時間が負の場合、shouldContinue() がすぐに false を返すかテスト
  TEST(MotionTimeContinuationConditionTest, MinusTargetTime)
  {
    Robot robot;
    int targetTime = -10;

    MotionTimeContinuationCondition condition(robot, targetTime);
    condition.prepare();

    EXPECT_FALSE(condition.shouldContinue());
  }

}  // namespace etrobocon2026_test
