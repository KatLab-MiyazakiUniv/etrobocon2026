/**
 * @file ClockUtilTest.cpp
 * @brief ClockUtilクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "ClockUtil.h"

namespace etrobocon2026_test {
  // ClockUtil::now()のテスト
  TEST(ClockUtilTest, Now)
  {
    int before = ClockUtil::now();
    ClockUtil::sleep(100);  // 100ミリ秒待機
    int after = ClockUtil::now();
    EXPECT_GE(after, before + 100);
  }

  // ClockUtil::sleep() のテスト
  TEST(ClockUtilTest, Sleep)
  {
    int before = ClockUtil::now();
    ClockUtil::sleep(200);  // 200ミリ秒待機
    int after = ClockUtil::now();
    EXPECT_GE(after, before + 200);
  }
}  // namespace etrobocon2026_test