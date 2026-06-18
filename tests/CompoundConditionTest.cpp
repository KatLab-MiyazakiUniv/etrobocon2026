/**
 * @file CompoundConditionTest.cpp
 * @brief CompoundConditionクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "CompoundCondition.h"
#include "DistanceCondition.h"
#include "RepeatCountCondition.h"
#include "BaseContinuationCondition.h"
#include "SensorColorCondition.h"
#include "MockNetworkSystem.h"

namespace etrobocon2026_test {

  // どちらの条件も満たさない場合は(AND:継続判定,OR:継続判定)になることを確認
  TEST(CompoundConditionTest, NeitherConditionMet)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetDistance = 1000.0;
    int targetRepeatCount = 5;

    // ANDの場合
    CompoundCondition::LogicalOperator logicalOperatorAnd = CompoundCondition::LogicalOperator::AND;

    CompoundCondition andCondition(
        robot, std::make_unique<DistanceCondition>(robot, targetDistance),
        std::make_unique<RepeatCountCondition>(robot, targetRepeatCount), logicalOperatorAnd);

    andCondition.prepare();

    EXPECT_TRUE(andCondition.shouldContinue());

    // ORの場合
    CompoundCondition::LogicalOperator logicalOperatorOr = CompoundCondition::LogicalOperator::OR;

    CompoundCondition orCondition(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                                  std::make_unique<RepeatCountCondition>(robot, targetRepeatCount),
                                  logicalOperatorOr);

    orCondition.prepare();

    EXPECT_TRUE(orCondition.shouldContinue());
  }

  // どちらかの条件のみ満たす場合は(AND:停止判定, OR:継続判定)になることを確認
  TEST(CompoundConditionTest, OnlyOneConditionMet)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    double targetDistance = 1000.0;
    int targetRepeatCount = 5;

    // ANDの場合
    CompoundCondition::LogicalOperator logicalOperatorAnd = CompoundCondition::LogicalOperator::AND;

    auto repeatCountConditionAnd = std::make_unique<RepeatCountCondition>(robot, targetRepeatCount);
    repeatCountConditionAnd->prepare();
    while(repeatCountConditionAnd->shouldContinue()) {
    }

    CompoundCondition andCondition(robot,
                                   std::make_unique<DistanceCondition>(robot, targetDistance),
                                   std::move(repeatCountConditionAnd), logicalOperatorAnd);

    andCondition.prepare();

    EXPECT_TRUE(andCondition.shouldContinue());

    // ORの場合
    CompoundCondition::LogicalOperator logicalOperatorOr = CompoundCondition::LogicalOperator::OR;

    auto repeatCountConditionOr = std::make_unique<RepeatCountCondition>(robot, targetRepeatCount);
    repeatCountConditionOr->prepare();
    while(repeatCountConditionOr->shouldContinue()) {
    }

    CompoundCondition orCondition(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                                  std::move(repeatCountConditionOr), logicalOperatorOr);

    orCondition.prepare();

    EXPECT_FALSE(orCondition.shouldContinue());
  }

  // 両方の条件を満たす場合は(AND:停止判定, OR:停止判定)になることを確認
  TEST(CompoundConditionTest, BothConditionsMet)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    ColorSensorController::COLOR targetColor = ColorSensorController::COLOR::BLACK;
    int targetRepeatCount = 5;

    // 目標色を設定
    spikeapi::ColorSensor::HSV targetHSV = { 1, 0, 5 };  // 黒色のHSV値
    ColorSensor::setHSV(targetHSV);

    // ANDの場合
    CompoundCondition::LogicalOperator logicalOperatorAnd = CompoundCondition::LogicalOperator::AND;

    auto repeatCountConditionAnd = std::make_unique<RepeatCountCondition>(robot, targetRepeatCount);
    repeatCountConditionAnd->prepare();
    while(repeatCountConditionAnd->shouldContinue()) {
    }

    CompoundCondition andCondition(robot,
                                   std::make_unique<SensorColorCondition>(robot, targetColor),
                                   std::move(repeatCountConditionAnd), logicalOperatorAnd);

    andCondition.prepare();

    EXPECT_TRUE(andCondition.shouldContinue());

    // ORの場合
    CompoundCondition::LogicalOperator logicalOperatorOr = CompoundCondition::LogicalOperator::OR;

    auto repeatCountConditionOr = std::make_unique<RepeatCountCondition>(robot, targetRepeatCount);
    repeatCountConditionOr->prepare();
    while(repeatCountConditionOr->shouldContinue()) {
    }

    CompoundCondition orCondition(robot, std::make_unique<SensorColorCondition>(robot, targetColor),
                                  std::move(repeatCountConditionOr), logicalOperatorOr);

    orCondition.prepare();

    EXPECT_FALSE(orCondition.shouldContinue());
  }

}  // namespace etrobocon2026_test