/**
 * @file DistanceConditionTest.cpp
 * @brief DistanceConditionクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "DistanceCondition.h"
#include "MockNetworkSystem.h"

namespace etrobocon2026_test {

  /**
   * prepare()直後は移動距離が0なので継続判定になることを確認
   */
  TEST(DistanceConditionTest, ContinueImmediatelyAfterPrepare)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    DistanceCondition condition(robot, 100.0);

    condition.prepare();

    EXPECT_TRUE(condition.shouldContinue());
  }

  /**
   * 目標距離に到達していない場合は継続判定になることを確認
   */
  TEST(DistanceConditionTest, ContinueBeforeTargetDistance)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    DistanceCondition condition(robot, 1000.0);

    condition.prepare();

    // 目標値に到達しない程度に前進
    robot.getWheelMotorControllerInstance().setRightSpeed(10);
    robot.getWheelMotorControllerInstance().setLeftSpeed(10);

    EXPECT_TRUE(condition.shouldContinue());
  }

  /**
   * 走行距離が目標距離に到達した場合は停止判定になることを確認
   */
  TEST(DistanceConditionTest, StopAtTargetDistance)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetDistance = 10.0;

    DistanceCondition condition(robot, targetDistance);

    condition.prepare();

    // 目標距離に到達するまで前進
    while(condition.shouldContinue()) {
      robot.getWheelMotorControllerInstance().setRightSpeed(50);
      robot.getWheelMotorControllerInstance().setLeftSpeed(50);
    }

    // 実際の走行距離を計算
    double actualMileage
        = fabs(Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                         robot.getWheelMotorControllerInstance().getLeftCount()));

    // 走行距離が目標距離以上であることを確認
    EXPECT_LE(targetDistance, actualMileage);
  }

  /**
   * 後退方向に移動した場合でも距離差分で停止判定できることを確認
   */
  TEST(DistanceConditionTest, StopAtTargetDistance2)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetDistance = 10.0;

    DistanceCondition condition(robot, targetDistance);

    condition.prepare();

    // 目標距離に到達するまで後退
    while(condition.shouldContinue()) {
      robot.getWheelMotorControllerInstance().setRightSpeed(-50);
      robot.getWheelMotorControllerInstance().setLeftSpeed(-50);
    }

    // 実際の走行距離を計算
    double actualMileage
        = fabs(Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                         robot.getWheelMotorControllerInstance().getLeftCount()));

    // 走行距離が目標距離以上であることを確認
    EXPECT_LE(targetDistance, actualMileage);
  }
  /**
   * 目標距離が0の場合は即停止判定になることを確認
   */
  TEST(DistanceConditionTest, ZeroTargetDistance)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    DistanceCondition condition(robot, 0.0);

    condition.prepare();

    EXPECT_FALSE(condition.shouldContinue());
  }

  /**
   * 負の目標距離を指定した場合は即停止判定になることを確認
   */
  TEST(DistanceConditionTest, NegativeTargetDistance)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    DistanceCondition condition(robot, -100.0);

    condition.prepare();

    EXPECT_FALSE(condition.shouldContinue());
  }

  /**
   * shouldContinue()を連続呼び出ししても異常終了しないことを確認
   */
  TEST(DistanceConditionTest, ContinuousCallStable)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    DistanceCondition condition(robot, 100.0);

    condition.prepare();

    for(int i = 0; i < 100; i++) {
      EXPECT_NO_THROW(condition.shouldContinue());
    }
  }

}  // namespace etrobocon2026_test