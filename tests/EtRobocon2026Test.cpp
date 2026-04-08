/**
 * @file   EtRobocon2026Test.cpp
 * @brief  EtRobocon2026クラスのテスト
 * @author miyahara046
 */

#include <gtest/gtest.h>
#include "EtRobocon2026.h"
#include "helpers/OStreamCapture.h"

using namespace std;

namespace etrobocon2026_test {
  // start関数が最終的に標準出力に"Hello KATLAB"を出力することを確認するテスト
  TEST(EtRobocon2026Test, Start)
  {
    string expected = "Hello KATLAB\n";

    // 標準出力をキャプチャ
    OStreamCapture capture(cout);

    // start関数を実行
    EtRobocon2026::start();

    EXPECT_EQ(expected, capture.getOutput());
  }
}  // namespace etrobocon2025_test