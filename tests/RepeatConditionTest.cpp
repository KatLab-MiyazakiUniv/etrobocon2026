/**
 * @file RepeatConditionTest.cpp
 * @brief RepeatConditionクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "RepeatCountCondition.h"

namespace etrobocon2026_test {

  // 目標回数に到達していない場合は継続判定になることを確認
  TEST(RepeatCountConditionTest, NotReachTargetCount)
  {
    Robot robot;

    RepeatCountCondition condition(robot, 100);

    condition.prepare();

    EXPECT_TRUE(condition.shouldContinue());
  }

  // 目標回数に到達した場合は停止判定になることを確認
  TEST(RepeatCountConditionTest, AfterTargetCount)
  {
    Robot robot;

    int targetCount = 100;
    int actualCount = 0;

    RepeatCountCondition condition(robot, targetCount);

    condition.prepare();

    // 目標回数に到達するまで動作
    while(condition.shouldContinue()) {
      actualCount++;
    }

    // 目標回数に到達するまで回っていることを確認
    EXPECT_EQ(targetCount, actualCount);
  }

  /**
   * 目標回数が0の場合は即停止判定になることを確認
   */
  TEST(RepeatCountConditionTest, ZeroTargetCount)
  {
    Robot robot;

    RepeatCountCondition condition(robot, 0);

    condition.prepare();

    EXPECT_FALSE(condition.shouldContinue());
  }

  /**
   * 目標回数が負の場合は即停止判定になることを確認
   */
  TEST(RepeatCountConditionTest, NegativeTargetCount)
  {
    Robot robot;

    RepeatCountCondition condition(robot, -1);

    condition.prepare();

    EXPECT_FALSE(condition.shouldContinue());
  }

}  // namespace etrobocon2026_test