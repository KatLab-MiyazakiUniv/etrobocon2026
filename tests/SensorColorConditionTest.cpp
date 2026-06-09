/**
 * @file SensorColorConditionTest.cpp
 * @brief SensorColorConditionクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "SensorColorCondition.h"
#include "ColorSensor.h"
#include <iostream>

namespace etrobocon2026_test {

  // 指定色を検知していない場合は継続判定になることを確認
  TEST(SensorColorConditionTest, NotReachTargetColor)
  {
    Robot robot;
    ColorSensorController::COLOR targetColor = ColorSensorController::COLOR::RED;

    SensorColorCondition condition(robot, targetColor);

    condition.prepare();

    EXPECT_TRUE(condition.shouldContinue());
    EXPECT_TRUE(condition.shouldContinue());
  }

  // 指定色を2回連続で検知した場合は停止判定になることを確認
  TEST(SensorColorConditionTest, TwoTimesTargetColor)
  {
    Robot robot;
    ColorSensorController::COLOR targetColor = ColorSensorController::COLOR::RED;

    SensorColorCondition condition(robot, targetColor);

    condition.prepare();

    int actualCount = 0;
    int expectedCount = 3;

    // 指定色のHSV値を設定
    spikeapi::ColorSensor::HSV targetHSV = { 0, 100, 100 };  // 赤色のHSV値
    ColorSensor::setHSV(targetHSV);

    EXPECT_TRUE(condition.shouldContinue());
    EXPECT_FALSE(condition.shouldContinue());
  }

  // 指定色がNONEの場合は停止判定になることを確認
  TEST(SensorColorConditionTest, NoneTargetColor)
  {
    Robot robot;
    ColorSensorController::COLOR targetColor = ColorSensorController::COLOR::NONE;

    SensorColorCondition condition(robot, targetColor);

    condition.prepare();

    EXPECT_FALSE(condition.shouldContinue());
  }

}  // namespace etrobocon2026_test