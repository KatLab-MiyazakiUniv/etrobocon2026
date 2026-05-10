/**
 * @file StraightTest.cpp
 * @brief Straightクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "Straight.h"

namespace etrobocon2026_test {
  // StraightクラスのexecuteStep()のテスト
  TEST(StraightTest, ExecuteStep)
  {
    // テスト用のRobotクラスのモックを作成
    class MockRobot : public Robot {
     public:
      MockRobot() : Robot() {}

      // モーターにセットされたPower値を記録するための変数
      double rightPowerSet = 0.0;
      double leftPowerSet = 0.0;

      // WheelMotorControllerのモッククラス
      class MockWheelMotorController : public WheelMotorController {
       public:
        MockWheelMotorController(MockRobot& parent) : WheelMotorController(parent) {}

        void setRightPower(double power) override { parent.rightPowerSet = power; }
        void setLeftPower(double power) override { parent.leftPowerSet = power; }

       private:
        MockRobot& parent;
      };

      // モックのWheelMotorControllerインスタンスを返すようにオーバーライド
      WheelMotorController& getWheelMotorControllerInstance() override
      {
        return mockWheelMotorController;
      }

     private:
      MockWheelMotorController mockWheelMotorController{ *this };
    };

    MockRobot mockRobot;

    // 動作を継続する条件を常にtrueにするモックのBaseContinuationConditionクラス
    class MockContinuationCondition : public BaseContinuationCondition {
     public:
      MockContinuationCondition(Robot& robot) : BaseContinuationCondition(robot) {}
      bool shouldContinue() override { return true; }
    };

    auto continuationCondition = std::make_unique<MockContinuationCondition>(mockRobot);
    Straight straight(mockRobot, std::move(continuationCondition), 100.0);

    straight.executeStep();

    // Power値が目標速度に相当する値になっているかをアサーション
    EXPECT_NEAR(mockRobot.rightPowerSet, 100.0, 10.0);  // 許容誤差10.0
    EXPECT_NEAR(mockRobot.leftPowerSet, 100.0, 10.0);   // 許容誤差10.0
  }
  {
  }
}  // namespace etrobocon2026_test
