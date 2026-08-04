/**
 * @file   BenchmarkStatisticsTest.cpp
 * @brief  BenchmarkStatisticsクラスをテストする
 * @author HaruArima08
 */

#include <gtest/gtest.h>
#include "BenchmarkStatistics.h"
#include <vector>

namespace etrobocon2026_test {

  // 標本が奇数個の場合、各統計量が正しく算出されることを確認する
  TEST(BenchmarkStatisticsTest, CalculateWithOddSamples)
  {
    std::vector<uint64_t> samples = { 1, 2, 3, 4, 5 };
    StatisticsResult actual = BenchmarkStatistics::calculate(samples);

    size_t expectedCount = 5;
    EXPECT_EQ(expectedCount, actual.count);
    EXPECT_DOUBLE_EQ(3.0, actual.mean);
    EXPECT_DOUBLE_EQ(3.0, actual.median);
    EXPECT_EQ(static_cast<uint64_t>(1), actual.min);
    EXPECT_EQ(static_cast<uint64_t>(5), actual.max);
  }

  // 標本が偶数個の場合、中央値が中央2要素の平均になることを確認する
  TEST(BenchmarkStatisticsTest, MedianWithEvenSamples)
  {
    std::vector<uint64_t> samples = { 1, 2, 3, 4 };
    StatisticsResult actual = BenchmarkStatistics::calculate(samples);

    EXPECT_DOUBLE_EQ(2.5, actual.median);
  }

  // 並べ替えられていない標本でも正しく算出されることを確認する
  TEST(BenchmarkStatisticsTest, CalculateWithUnsortedSamples)
  {
    std::vector<uint64_t> samples = { 50, 10, 30, 20, 40 };
    StatisticsResult actual = BenchmarkStatistics::calculate(samples);

    EXPECT_DOUBLE_EQ(30.0, actual.mean);
    EXPECT_DOUBLE_EQ(30.0, actual.median);
    EXPECT_EQ(static_cast<uint64_t>(10), actual.min);
    EXPECT_EQ(static_cast<uint64_t>(50), actual.max);
  }

  // 不偏標準偏差が正しく算出されることを確認する
  TEST(BenchmarkStatisticsTest, StandardDeviationIsUnbiased)
  {
    // 平均5、偏差の二乗和36、不偏分散36/(5-1)=9なので標準偏差は3になる
    // 母標準偏差ならsqrt(36/5)=2.683...となるため、不偏かどうかを区別できる
    std::vector<uint64_t> samples = { 2, 2, 5, 8, 8 };
    StatisticsResult actual = BenchmarkStatistics::calculate(samples);

    EXPECT_DOUBLE_EQ(5.0, actual.mean);
    EXPECT_DOUBLE_EQ(3.0, actual.standardDeviation);
  }

  // 標本が1つの場合、標準偏差が0になることを確認する
  TEST(BenchmarkStatisticsTest, StandardDeviationIsZeroWithSingleSample)
  {
    std::vector<uint64_t> samples = { 42 };
    StatisticsResult actual = BenchmarkStatistics::calculate(samples);

    EXPECT_DOUBLE_EQ(0.0, actual.standardDeviation);
    EXPECT_DOUBLE_EQ(42.0, actual.mean);
    EXPECT_DOUBLE_EQ(42.0, actual.median);
  }

  // 標本が空の場合、全ての統計量が0になることを確認する
  TEST(BenchmarkStatisticsTest, CalculateWithEmptySamples)
  {
    std::vector<uint64_t> samples;
    StatisticsResult actual = BenchmarkStatistics::calculate(samples);

    size_t expectedCount = 0;
    EXPECT_EQ(expectedCount, actual.count);
    EXPECT_DOUBLE_EQ(0.0, actual.mean);
    EXPECT_DOUBLE_EQ(0.0, actual.median);
    EXPECT_EQ(static_cast<uint64_t>(0), actual.min);
    EXPECT_EQ(static_cast<uint64_t>(0), actual.max);
    EXPECT_DOUBLE_EQ(0.0, actual.standardDeviation);
  }

  // 呼び出し元の配列が変更されないことを確認する
  TEST(BenchmarkStatisticsTest, SamplesAreNotModified)
  {
    std::vector<uint64_t> samples = { 3, 1, 2 };
    BenchmarkStatistics::calculate(samples);

    EXPECT_EQ(static_cast<uint64_t>(3), samples[0]);
    EXPECT_EQ(static_cast<uint64_t>(1), samples[1]);
    EXPECT_EQ(static_cast<uint64_t>(2), samples[2]);
  }

}  // namespace etrobocon2026_test
