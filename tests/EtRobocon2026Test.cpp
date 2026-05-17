/**
 * @file   EtRobocon2026Test.cpp
 * @brief  EtRobocon2026クラスのテスト
 * @author miyahara046
 */
/*
#include <gtest/gtest.h>
#include "EtRobocon2026.h"

namespace etrobocon2026_test {
  // start関数が最終的に標準出力に"Hello KATLAB"を出力することを確認するテスト
  TEST(EtRobocon2026Test, Start)
  {
    std::string expected = "Hello KATLAB\n";

    // 標準出力をキャプチャ開始
    testing::internal::CaptureStdout();

    // start関数を実行
    EtRobocon2026::start();

    // キャプチャした出力を取得
    std::string output = testing::internal::GetCapturedStdout();

    EXPECT_EQ(expected, output);
  }
}  // namespace etrobocon2026_test
*/
#include <gtest/gtest.h>
#include "EtRobocon2026.h"
#include "Rotation.h"

namespace etrobocon2026_test {

  // =====================================================
  // EtRobocon2026 startテスト
  // =====================================================
  TEST(EtRobocon2026Test, Start)
  {
    std::string expected = "Hello KATLAB\n";

    testing::internal::CaptureStdout();

    EtRobocon2026::start();

    std::string output = testing::internal::GetCapturedStdout();

    EXPECT_EQ(expected, output);
  }

  // =====================================================
  // Rotationテスト用ダミー
  // =====================================================
  class DummyContinuationCondition : public BaseContinuationCondition {
   public:
    DummyContinuationCondition(Robot& robot) : BaseContinuationCondition(robot) {}

    bool isContinue() override { return true; }
  };

  // テスト用Rotation
  class TestRotation : public Rotation {
   public:
    TestRotation(Robot& robot, std::unique_ptr<BaseContinuationCondition> cond, double target)
      : Rotation(robot, std::move(cond)), targetAngle(target)
    {
    }

    double getTargetAngle() override { return targetAngle; }

   private:
    double targetAngle;
  };

  // =====================================================
  // ★80度回転テスト
  // =====================================================
  TEST(RotationTest, Rotate80Degrees_TargetCheck)
  {
    Robot robot;

    // 初期角度固定
    robot.getIMUControllerInstance().setAzimuth(0.0);

    TestRotation rotation(robot, std::make_unique<DummyContinuationCondition>(robot),
                          80.0  // ★80度回転
    );

    EXPECT_DOUBLE_EQ(80.0, rotation.getTargetAngle());
  }

  // =====================================================
  // ★80度回転（方向チェック）
  // =====================================================
  TEST(RotationTest, Rotate80Degrees_Direction)
  {
    Robot robot;

    robot.getIMUControllerInstance().setAzimuth(0.0);

    TestRotation rotation(robot, std::make_unique<DummyContinuationCondition>(robot), 80.0);

    rotation.executeStep();

    double right = robot.getWheelMotorControllerInstance().getRightPower();
    double left = robot.getWheelMotorControllerInstance().getLeftPower();

    EXPECT_GT(left, 0);   // 左前進
    EXPECT_LT(right, 0);  // 右後退（右回転）
  }

}  // namespace etrobocon2026_test