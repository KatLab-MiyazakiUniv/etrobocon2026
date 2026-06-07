/**
 * @file CompoundContinuationConditionTest.cpp
 * @brief CompoundContinuationConditionクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "CompoundContinuationCondition.h"
#include "DistanceCondition.h"
#include "RepeatCountContinuationCondition.h"
#include "BaseContinuationCondition.h"
#include "ColorContinuationCondition.h"

namespace etrobocon2026_test {

  // どちらの条件も満たさない場合は(AND:継続判定,OR:継続判定)になることを確認
  TEST(CompoundContinuationConditionTest, NeitherConditionMet)
  {
    Robot robot;
    double targetDistance = 1000.0;
    int targetRepeatCount = 5;

    std::unique_ptr<BaseContinuationCondition> distanceCondition
        = std::make_unique<DistanceCondition>(robot, targetDistance);
    std::unique_ptr<BaseContinuationCondition> repeatCountCondition
        = std::make_unique<RepeatCountContinuationCondition>(robot, targetRepeatCount);

    // ANDの場合
    CompoundContinuationCondition::LogicalOperator logicalOperatorAnd
        = CompoundContinuationCondition::LogicalOperator::AND;

    CompoundContinuationCondition andCondition(robot, std::move(distanceCondition),
                                               std::move(repeatCountCondition), logicalOperatorAnd);

    andCondition.prepare();

    EXPECT_TRUE(andCondition.shouldContinue());

    // ORの場合
    CompoundContinuationCondition::LogicalOperator logicalOperatorOr
        = CompoundContinuationCondition::LogicalOperator::OR;

    CompoundContinuationCondition orCondition(robot, std::move(distanceCondition),
                                              std::move(repeatCountCondition), logicalOperatorOr);

    orCondition.prepare();

    EXPECT_TRUE(orCondition.shouldContinue());
  }

  // どちらかの条件のみ満たす場合は(AND:停止判定, OR:継続判定)になることを確認
  TEST(CompoundContinuationConditionTest, OnlyOneConditionMet)
  {
    Robot robot;
    double targetDistance = 1000.0;
    int targetRepeatCount = 5;

    std::unique_ptr<BaseContinuationCondition> distanceCondition
        = std::make_unique<DistanceCondition>(robot, targetDistance);
    std::unique_ptr<BaseContinuationCondition> repeatCountCondition
        = std::make_unique<RepeatCountContinuationCondition>(robot, targetRepeatCount);

    repeatCountCondition->prepare();

    // 目標回数に到達するまで動作
    while(repeatCountCondition->shouldContinue()) {
    }

    // ANDの場合
    CompoundContinuationCondition::LogicalOperator logicalOperatorAnd
        = CompoundContinuationCondition::LogicalOperator::AND;

    CompoundContinuationCondition andCondition(robot, std::move(distanceCondition),
                                               std::move(repeatCountCondition), logicalOperatorAnd);

    andCondition.prepare();

    EXPECT_TRUE(andCondition.shouldContinue());

    // ORの場合
    CompoundContinuationCondition::LogicalOperator logicalOperatorOr
        = CompoundContinuationCondition::LogicalOperator::OR;

    CompoundContinuationCondition orCondition(robot, std::move(distanceCondition),
                                              std::move(repeatCountCondition), logicalOperatorOr);

    orCondition.prepare();

    EXPECT_FALSE(orCondition.shouldContinue());
  }

  // 両方の条件を満たす場合は(AND:停止判定, OR:停止判定)になることを確認
  TEST(CompoundContinuationConditionTest, BothConditionsMet)
  {
    Robot robot;
    ColorSensorController::COLOR targetColor = ColorSensorController::COLOR::BLACK;
    int targetRepeatCount = 5;

    std::unique_ptr<BaseContinuationCondition> distanceCondition
        = std::make_unique<ColorContinuationCondition>(robot, targetColor);
    std::unique_ptr<BaseContinuationCondition> repeatCountCondition
        = std::make_unique<RepeatCountContinuationCondition>(robot, targetRepeatCount);

    // 目標色を設定
    spikeapi::ColorSensor::HSV targetHSV = { 1, 0, 5 };  // 黒色のHSV値
    ColorSensor::setHSV(targetHSV);

    // 目標回数に到達するまで動作
    repeatCountCondition->prepare();
    while(repeatCountCondition->shouldContinue()) {
    }

    // ANDの場合
    CompoundContinuationCondition::LogicalOperator logicalOperatorAnd
        = CompoundContinuationCondition::LogicalOperator::AND;

    CompoundContinuationCondition andCondition(robot, std::move(distanceCondition),
                                               std::move(repeatCountCondition), logicalOperatorAnd);

    andCondition.prepare();

    EXPECT_TRUE(andCondition.shouldContinue());

    // ORの場合
    CompoundContinuationCondition::LogicalOperator logicalOperatorOr
        = CompoundContinuationCondition::LogicalOperator::OR;

    CompoundContinuationCondition orCondition(robot, std::move(distanceCondition),
                                              std::move(repeatCountCondition), logicalOperatorOr);

    orCondition.prepare();

    EXPECT_FALSE(orCondition.shouldContinue());
  }

}  // namespace etrobocon2026_test