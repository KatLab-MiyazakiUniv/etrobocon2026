#include <gtest/gtest.h>
#include "Pid.h"  // スクリーンショット (28).jpg のクラス宣言

// テスト用のフィクスチャ（共通の初期化処理）
class PidTest : public ::testing::Test {
 protected:
  Pid* pid;
  void SetUp() override
  {
    // P=1.0, I=0.0, D=0.0, Target=100.0 で初期化
    pid = new Pid(1.0, 0.0, 0.0, 100.0);
  }
  void TearDown() override { delete pid; }
};

// 1. 比例項 (P) の計算テスト
TEST_F(PidTest, ProportionalControlTest)
{
  // 偏差 = 100.0 - 90.0 = 10.0
  // P項 = 1.0 * 10.0 = 10.0
  double output = pid->calculatePid(90.0, 0.01);
  EXPECT_NEAR(output, 10.0, 1e-5);
}

// 2. 偏差ゼロでの出力テスト
TEST_F(PidTest, ZeroDeviationTest)
{
  // 偏差 = 100.0 - 100.0 = 0.0
  double output = pid->calculatePid(100.0, 0.01);
  EXPECT_DOUBLE_EQ(output, 0.0);
}

// 3. 積分項 (I) の累積と制限 (Anti-windup) のテスト
TEST_F(PidTest, IntegralWindupTest)
{
  // ゲイン設定: P=0, I=1.0, D=0
  pid->setPidGain(0.0, 1.0, 0.0);

  // 長時間、偏差がある状態をシミュレート
  for(int i = 0; i < 1000; ++i) {
    pid->calculatePid(50.0, 0.1);
  }

  // 画像内の maxIntegral は 100.0 と定義されている
  // Iゲインが 1.0 なので、出力は maxIntegral (100.0) で止まるはず
  double output = pid->calculatePid(50.0, 0.1);
  EXPECT_LE(output, 100.0);
  EXPECT_GE(output, -100.0);
}

// 4. 微分項 (D) のフィルタ動作テスト
TEST_F(PidTest, DerivativeFilterTest)
{
  // ゲイン設定: P=0, I=0, D=1.0
  pid->setPidGain(0.0, 0.0, 1.0);

  // 1回目：偏差10 (100-90)
  pid->calculatePid(90.0, 0.1);

  // 2回目：偏差0 (100-100) -> 急激な変化
  // フィルタ(alpha=0.8)があるため、出力は一気に0にならず、前回の値を引きずる
  double output = pid->calculatePid(100.0, 0.1);

  // フィルタなしなら (0 - 10)/0.1 = -100 になるはずだが、
  // alphaの影響でそれよりも緩やかな数値になることを確認
  EXPECT_GT(output, -100.0);
}

// 5. 不正なゲイン設定に対するガードテスト
TEST_F(PidTest, NegativeGainGuardTest)
{
  // 負の値を設定しようとする
  pid->setPidGain(-5.0, -1.0, -2.0);

  // 画像内の実装では負の値を無視するため、
  // 前回の有効な値（SetupでのP=1.0）が維持されているか確認
  double output = pid->calculatePid(90.0, 0.01);
  EXPECT_GT(output, 0.0);
}