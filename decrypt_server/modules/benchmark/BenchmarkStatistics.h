/**
 * @file   BenchmarkStatistics.h
 * @brief  計測値の統計量を算出するクラス
 * @author HaruArima08
 */

#ifndef BENCHMARK_STATISTICS_H
#define BENCHMARK_STATISTICS_H

#include <cstdint>
#include <vector>

/**
 * @brief 計測値の統計量
 */
struct StatisticsResult {
  size_t count = 0;                // 標本数
  double mean = 0.0;               // 平均値
  double median = 0.0;             // 中央値
  uint64_t min = 0;                // 最小値
  uint64_t max = 0;                // 最大値
  double standardDeviation = 0.0;  // 不偏標準偏差
};

class BenchmarkStatistics {
 public:
  /**
   * @brief 計測値から統計量を算出する
   * @param samples 計測値の配列
   * @return StatisticsResult 統計量（標本が空の場合は全て0）
   * @note 中央値は標本数が偶数の場合、中央2要素の平均とする
   *       標準偏差は標本標準偏差ではなく不偏標準偏差(n-1で除算)とする
   */
  static StatisticsResult calculate(const std::vector<uint64_t>& samples);

 private:
  BenchmarkStatistics();  // インスタンス化禁止
};

#endif  // BENCHMARK_STATISTICS_H
