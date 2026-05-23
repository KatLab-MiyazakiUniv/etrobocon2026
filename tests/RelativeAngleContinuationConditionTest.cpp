/**
 * @file RelativeAngleContinuationConditionTest.cpp
 * @brief RelativeAngleContinuationConditionクラスをテストする
 * @author okuyama0528 yutaro-1214
 */

#include <gtest/gtest.h>
#include "AngleContinuationCondition.h"

namespace etrobocon2026_test {
  /**
   * prepare()直後は移動角度が0なので継続判定になることを確認
   */
  TEST(AngleContinuationConditionConditionTest, AngleContinuationConditionConditionAfterPrepare)
  {
    Robot robot;

    AngleContinuationCondition condition(robot, 100.0);

    condition.prepare();

    EXPECT_TRUE(condition.shouldContinue());
  }

  /**
   * 目標角度に到達していない場合は継続判定になることを確認
   */
  TEST(AngleContinuationConditionConditionTest, NotReachTargetAngle)
  {
    Robot robot;

    AngleContinuationCondition condition(robot, 100.0);

    condition.prepare();

    for(int i = 0; i < 10; i++) {
      robot.getIMUControllerInstance().getAzimuth();
    }
    EXPECT_TRUE(condition.shouldContinue());
  }

  /**
   * 角度が目標角度に到達した場合は停止判定になることを確認
   */
  TEST(AngleContinuationConditionConditionTest, AfterTargetAngle)
  {
    Robot robot;

    double targetAngle = 20;
    // initialAngleとconditionで２回getAzimuthを実行するため初期値２
    int i = 2;

    double initialAngle = robot.getIMUControllerInstance().getAzimuth();

    AngleContinuationCondition condition(robot, targetAngle);

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
  TEST(AngleContinuationConditionConditionTest, ZeroTargetAbgle)
  {
    Robot robot;

    AngleContinuationCondition condition(robot, 0.0);

    condition.prepare();

    EXPECT_FALSE(condition.shouldContinue());
  }

  /**
   * shouldContinue()を連続呼び出ししても異常終了しないことを確認
   */
  TEST(AngleContinuationConditionConditionTest, ContinuousCallStable)
  {
    Robot robot;

    AngleContinuationCondition condition(robot, 100.0);

    condition.prepare();

    for(int i = 0; i < 100; i++) {
      EXPECT_NO_THROW(condition.shouldContinue());
    }
  }

}  // namespace etrobocon2026_test