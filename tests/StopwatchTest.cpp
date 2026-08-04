/**
 * @file   StopwatchTest.cpp
 * @brief  Stopwatchクラスをテストする
 * @author HaruArima08
 */

#include <gtest/gtest.h>
#include "Stopwatch.h"
#include "ClockUtil.h"

namespace etrobocon2026_test {

  // 経過時間が単調非減少であることを確認する
  TEST(StopwatchTest, ElapsedMicroIsMonotonic)
  {
    Stopwatch stopwatch;
    uint64_t first = stopwatch.elapsedMicro();
    uint64_t second = stopwatch.elapsedMicro();

    EXPECT_LE(first, second);
  }

  // スリープした分だけ経過時間が進むことを確認する
  TEST(StopwatchTest, ElapsedMicroAdvancesAfterSleep)
  {
    Stopwatch stopwatch;
    ClockUtil::sleep(10);  // 10ミリ秒 = 10000マイクロ秒

    // クロックの誤差を考慮して、スリープ時間の8割以上進んでいれば良しとする
    uint64_t expected = 8000;
    EXPECT_GE(stopwatch.elapsedMicro(), expected);
  }

  // リセットすると経過時間が短くなることを確認する
  TEST(StopwatchTest, ResetRestartsMeasurement)
  {
    Stopwatch stopwatch;
    ClockUtil::sleep(10);
    uint64_t beforeReset = stopwatch.elapsedMicro();

    stopwatch.reset();
    uint64_t afterReset = stopwatch.elapsedMicro();

    EXPECT_LT(afterReset, beforeReset);
  }

}  // namespace etrobocon2026_test
