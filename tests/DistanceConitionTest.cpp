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

      // モーターのカウントを外部から操作するための変数
      int32_t rightCount = 0;
      int32_t leftCount = 0;

      // WheelMotorControllerのモッククラス
      class MockWheelMotorController : public WheelMotorController {
       public:
        /**
         * 修正ポイント:
         * 1. 親クラス WheelMotorController は引数なしで呼び出す
         * 2. 参照メンバ parent は初期化リストで p を代入して初期化する
         */
        MockWheelMotorController(MockRobot& p) : WheelMotorController(), parent(p) {}

        /**
         * 修正ポイント:
         * 基底クラスのメソッドに virtual が付いていないため、
         * 'override' キーワードを削除して「関数隠蔽」の形で定義します。
         */
        int32_t getRightCount() { return parent.rightCount; }
        int32_t getLeftCount() { return parent.leftCount; }

       private:
        MockRobot& parent;
      };

      /**
       * Robot.h 側の getWheelMotorControllerInstance が virtual であれば
       * ここは override 可能です。
       */
      WheelMotorController& getWheelMotorControllerInstance() { return mockWheelMotorController; }

     private:
      // 自分自身 (*this) を渡して MockWheelMotorController を初期化
      MockWheelMotorController mockWheelMotorController{ *this };
    };

    // --- テスト実行 ---
    MockRobot mockRobot;

    // 目標距離を 100.0 mm に設定
    DistanceCondition distanceCondition(mockRobot, 100.0);
    distanceCondition.prepare();

    // 1. 初期状態 (カウント0) では走行を継続すべき
    EXPECT_TRUE(distanceCondition.shouldContinue());

    // 2. 走行距離が目標に達していない状態
    mockRobot.rightCount = 10;
    mockRobot.leftCount = 10;
    EXPECT_TRUE(distanceCondition.shouldContinue());

    // 3. 走行距離が目標を超えるようにカウントを更新
    // (タイヤ径等によりますが、大きな値を入れて確実に停止させる例)
    mockRobot.rightCount = 1000;
    mockRobot.leftCount = 1000;

    // 走行終了を検知して false を返すはず
    EXPECT_FALSE(distanceCondition.shouldContinue());
  }

}  // namespace etrobocon2026_test