/**
 * @file ClockUtilTest.cpp
 * @brief ClockUtilクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "ClockUtil.h"

namespace etrobocon2026_test {
  // now()がsleep()で待機した時間だけ増加するかのテスト
  TEST(ClockUtilTest, NowSleep)
  {
    double start = ClockUtil::now();

    ClockUtil::sleep(10);

    double end = ClockUtil::now();

    EXPECT_GE(end, start + 0.01);
  }

  // now()がwait()で待機した時間だけ増加するかのテスト
  TEST(ClockUtilTest, NowWait)
  {
    double start = ClockUtil::now();

    ClockUtil::wait(20);

    double end = ClockUtil::now();
    EXPECT_GE(end, start + 0.02);
  }

  // now()が小数点以下の値を返すかのテスト
  TEST(ClockUtilTest, NowHasFraction)
  {
    double start = ClockUtil::now();

    ClockUtil::sleep(1);

    double end = ClockUtil::now();

    double diff = end - start;

    EXPECT_GT(diff, 0.0);
    EXPECT_LT(diff, 1.0);
  }

  // sleep()に0を渡したときに、now()がほとんど増加しないことを確認するテスト
  TEST(ClockUtilTest, SleepZero)
  {
    double start = ClockUtil::now();

    ClockUtil::sleep(0);

    double end = ClockUtil::now();

    EXPECT_LT(end - start, 0.01);
  }

  // sleep()に負の値を渡したときに、now()がほとんど増加しないことを確認するテスト
  TEST(ClockUtilTest, SleepNegative)
  {
    double start = ClockUtil::now();

    ClockUtil::sleep(-10);

    double end = ClockUtil::now();

    EXPECT_LT(end - start, 0.01);
  }

  // wait()に0を渡したときに、now()がほとんど増加しないことを確認するテスト
  TEST(ClockUtilTest, WaitZero)
  {
    double start = ClockUtil::now();

    ClockUtil::wait(0);

    double end = ClockUtil::now();

    EXPECT_LT(end - start, 0.01);
  }

  // wait()に負の値を渡したときに、now()がほとんど増加しないことを確認するテスト
  TEST(ClockUtilTest, WaitNegative)
  {
    double start = ClockUtil::now();

    ClockUtil::wait(-20);

    double end = ClockUtil::now();

    EXPECT_LT(end - start, 0.01);
  }

}  // namespace etrobocon2026_test