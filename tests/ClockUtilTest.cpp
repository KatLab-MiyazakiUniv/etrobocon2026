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

  // nowMicro()がマイクロ秒を返し、now()のミリ秒と整合することを確認するテスト
  TEST(ClockUtilTest, NowMicroIsConsistentWithNow)
  {
    uint64_t micro = ClockUtil::nowMicro();
    int milli = ClockUtil::now();

    // nowMicro()を先に取得しているので、ミリ秒へ丸めた値は必ずnow()以下になる
    EXPECT_LE(static_cast<int>(micro / 1000), milli);
  }

  // nowMicro()がsleep()した分だけ進むことを確認するテスト
  TEST(ClockUtilTest, NowMicroAdvancesAfterSleep)
  {
    uint64_t start = ClockUtil::nowMicro();

    ClockUtil::sleep(10);  // 10ミリ秒 = 10000マイクロ秒

    uint64_t end = ClockUtil::nowMicro();

    // クロックの誤差を考慮して、スリープ時間の8割以上進んでいれば良しとする
    EXPECT_GE(end - start, static_cast<uint64_t>(8000));
  }

}  // namespace etrobocon2026_test