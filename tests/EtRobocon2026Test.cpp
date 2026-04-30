/**
 * @file   EtRobocon2026Test.cpp
 * @brief  EtRobocon2026クラスのテスト
 * @author miyahara046
 */

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