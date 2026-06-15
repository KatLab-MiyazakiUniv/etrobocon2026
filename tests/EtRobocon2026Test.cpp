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
    // 標準出力をキャプチャ開始
    testing::internal::CaptureStdout();

    // start関数を実行
    // EtRobocon2026::start();
    Logger::info("Hello KATLAB");
    // キャプチャした出力を取得
    std::string output = testing::internal::GetCapturedStdout();

    EXPECT_NE(std::string::npos, output.find("[INFO] Hello KATLAB\n"));
  }
}  // namespace etrobocon2026_test