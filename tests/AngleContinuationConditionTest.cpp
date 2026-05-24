/**
 * @file AngleContinuationConditionTest.cpp
 * @brief AngleContinuationConditionクラスをテストする
 * @author okuyama0528 yutaro-1214
 */

#include <gtest/gtest.h>
#include "AngleContinuationCondition.h"
#include "AngleNormalizer.h"

namespace etrobocon2026_test {
  // 目標角度に到達していない場合は継続判定になることを確認
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

  // 角度が目標角度に到達した場合は停止判定になることを確認
  TEST(AngleContinuationConditionConditionTest, AfterTargetAngle)
  {
    Robot robot;

    double targetAngle = 20;
    // initialAngleとconditionで2回getAzimuthを実行するため初期値は2
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

  // shouldContinue()を連続呼び出ししても異常終了しないことを確認

  TEST(AngleContinuationConditionConditionTest, ContinuousCallStable)
  {
    Robot robot;

    AngleContinuationCondition condition(robot, 100.0);

    condition.prepare();

    for(int i = 0; i < 100; i++) {
      EXPECT_NO_THROW(condition.shouldContinue());
    }
  }

  // 取得した方位角が180°より大きいとき正しく正規化されるかのテスト
  TEST(AngleContinuationCondition, normalizeAnglePlus)
  {
    Robot robot;

    AngleContinuationCondition condition(robot, 0);

    double EXPECTED = 90;

    double angle = 450;

    EXPECT_EQ(EXPECTED, AngleNormalizer::NormalizeAngle(angle));
  }

  // 取得した方位角が-180°より小さいとき正しく正規化されるかのテスト
  TEST(AngleContinuationCondition, normalizeAngleMinus)
  {
    Robot robot;

    AngleContinuationCondition condition(robot, 0);

    double EXPECTED = -90;

    double angle = -450;

    EXPECT_EQ(EXPECTED, AngleNormalizer::NormalizeAngle(angle));
  }

  // 正しく誤差の判定がされるかのテスト
  TEST(AngleContinuationCondition, tolerance)
  {
    Robot robot;

    AngleContinuationCondition condition(robot, 3);

    EXPECT_FALSE(condition.shouldContinue());
  }

}  // namespace etrobocon2026_test