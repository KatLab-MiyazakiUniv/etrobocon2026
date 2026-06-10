/**
 * @file ClockUtilTest.cpp
 * @brief ClockUtilクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "ClockUtil.h"

namespace etrobocon2026_test {
  // sleep()後に取得した時間が指定した待機時間以上増加するかテスト
  TEST(ClockUtilTest, NowSleep)
  {
    int start = ClockUtil::now();
    int sleepTime = 10;

    ClockUtil::sleep(sleepTime);

    int end = ClockUtil::now();
    int expected = start + sleepTime;

    EXPECT_LE(expected, end);
  }

  // wait()後に取得した時間がwait()で待機した時間以上増加するかテスト
  TEST(ClockUtilTest, NowWait)
  {
    int start = ClockUtil::now();
    int waitTime = 20;

    ClockUtil::wait(waitTime);

    int end = ClockUtil::now();
    int expected = start + waitTime;

    EXPECT_LE(expected, end);
  }

  // sleep()に0を渡したときに、now()がほとんど増加しないことを確認するテスト
  TEST(ClockUtilTest, SleepZero)
  {
    int start = ClockUtil::now();

    ClockUtil::sleep(0);

    int end = ClockUtil::now();

    EXPECT_LE(end - start, 5);
  }

  // sleep()に負の値を渡したときに、now()がほとんど増加しないことを確認するテスト
  TEST(ClockUtilTest, SleepNegative)
  {
    int start = ClockUtil::now();

    ClockUtil::sleep(-10);

    int end = ClockUtil::now();

    EXPECT_LE(end - start, 1);
  }

  // wait()に0を渡したときに、now()がほとんど増加しないことを確認するテスト
  TEST(ClockUtilTest, WaitZero)
  {
    int start = ClockUtil::now();

    ClockUtil::wait(0);

    int end = ClockUtil::now();

    EXPECT_LE(end - start, 1);
  }

  // wait()に負の値を渡したときに、now()がほとんど増加しないことを確認するテスト
  TEST(ClockUtilTest, WaitNegative)
  {
    int start = ClockUtil::now();

    ClockUtil::wait(-20);

    int end = ClockUtil::now();

    EXPECT_LE(end - start, 5);
  }

}  // namespace etrobocon2026_test