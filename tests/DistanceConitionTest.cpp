/**
 * @file DistanceConditionTest.cpp
 * @brief DistanceConditionクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "DistanceCondition.h"

namespace etrobocon2026_test {
  // DistanceCondition::shouldContinue()のテスト
  TEST(DistanceConditionTest, ShouldContinue)
  {
    // テスト用のRobotクラスのモックを作成
    class MockRobot : public Robot {
     public:
      MockRobot() : Robot() {}

      // モーターにセットされたPower値を記録するための変数
      int32_t rightCount = 0;
      int32_t leftCount = 0;

      // WheelMotorControllerのモッククラス
      class MockWheelMotorController : public WheelMotorController {
       public:
        MockWheelMotorController(MockRobot& parent) : WheelMotorController(parent) {}

        int32_t getRightCount() override { return parent.rightCount; }
        int32_t getLeftCount() override { return parent.leftCount; }

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

    DistanceCondition distanceCondition(mockRobot, 100.0);
    distanceCondition.prepare();

    // 初期状態では shouldContinue() は true を返すはず
    EXPECT_TRUE(distanceCondition.shouldContinue());

    // 走行距離が目標距離に到達するようにカウントを増やす
    mockRobot.rightCount = 30;  // 約14.65mm
    mockRobot.leftCount = 40;   // 約19.54mm

    // shouldContinue() は false を返すはず
    EXPECT_FALSE(distanceCondition.shouldContinue());
  }
}  // namespace etrobocon2026_test