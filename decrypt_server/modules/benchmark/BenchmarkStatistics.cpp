/**
 * @file   BenchmarkStatistics.cpp
 * @brief  計測値の統計量を算出するクラス
 * @author HaruArima08
 */

#include "BenchmarkStatistics.h"
#include <algorithm>
#include <cmath>

StatisticsResult BenchmarkStatistics::calculate(const std::vector<uint64_t>& samples)
{
  StatisticsResult result;
  if(samples.empty()) return result;

  // 中央値の算出のために並べ替えるので、呼び出し元の配列は変更しないようコピーする
  std::vector<uint64_t> sorted = samples;
  std::sort(sorted.begin(), sorted.end());

  result.count = sorted.size();
  result.min = sorted.front();
  result.max = sorted.back();

  double sum = 0.0;
  for(uint64_t sample : sorted) sum += static_cast<double>(sample);
  result.mean = sum / static_cast<double>(result.count);

  size_t middle = result.count / 2;
  if(result.count % 2 == 0) {
    result.median
        = (static_cast<double>(sorted[middle - 1]) + static_cast<double>(sorted[middle])) / 2.0;
  } else {
    result.median = static_cast<double>(sorted[middle]);
  }

  // 標本が1つの場合は不偏分散が定義できないため、標準偏差は0とする
  if(result.count > 1) {
    double squaredSum = 0.0;
    for(uint64_t sample : sorted) {
      double difference = static_cast<double>(sample) - result.mean;
      squaredSum += difference * difference;
    }
    result.standardDeviation = std::sqrt(squaredSum / static_cast<double>(result.count - 1));
  }

  return result;
}
