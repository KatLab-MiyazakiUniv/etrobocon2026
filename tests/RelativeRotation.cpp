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

    AngleContinuationCondition condition(robot, 4000.0);

    condition.prepare();

    // 目標値に到達しない程度に前進
    robot.getWheelMotorControllerInstance().setRightSpeed(10);
    robot.getWheelMotorControllerInstance().setLeftSpeed(-10);

    EXPECT_TRUE(condition.shouldContinue());
  }

  /**
   * 角度が目標角度に到達した場合は停止判定になることを確認
   */
  TEST(AngleContinuationConditionConditionTest, AfterTargetAngle)
  {
    Robot robot;

    double targetAngle = 1;

    double initialAngle = robot.getIMUControllerInstance().getAzimuth();

    AngleContinuationCondition condition(robot, targetAngle);

    condition.prepare();

    // 目標角度に到達するまで回頭
    while(condition.shouldContinue()) {
      robot.getWheelMotorControllerInstance().setRightSpeed(50);
      robot.getWheelMotorControllerInstance().setLeftSpeed(-50);
    }

    // 実際の回頭角度を取得
    double endAngle = robot.getIMUControllerInstance().getAzimuth();

    // 回頭量（どれだけ回ったか）
    double actualAngle = fabs(endAngle - initialAngle);

    // 目標角度以上回っていることを確認
    EXPECT_GE(actualAngle, targetAngle);
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