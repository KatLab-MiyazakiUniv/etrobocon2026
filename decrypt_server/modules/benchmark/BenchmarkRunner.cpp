/**
 * @file   BenchmarkRunner.cpp
 * @brief  復号モードごとに性能を計測して比較するクラス
 * @author HaruArima08
 */

#include "BenchmarkRunner.h"
#include <cstdio>

BenchmarkRunner::BenchmarkRunner(const BenchmarkCondition& _condition,
                                 BenchmarkCsvWriter& _csvWriter)
  : condition(_condition), csvWriter(_csvWriter)
{
}

BenchmarkModeResult BenchmarkRunner::run(IDecryptService& service, const std::string& serverIp)
{
  BenchmarkModeResult result;
  result.mode = service.getMode();
  result.serverIp = serverIp;

  // OpenSSLの遅延初期化やTCPスロースタート等の影響を除くため、最初の数回は計測に含めない
  for(int i = 0; i < condition.warmupTrials; i++) {
    DecryptMeasurement warmup;
    service.decrypt(condition.key, condition.encryptedText, condition.iterations, warmup);
    uint64_t warmupPing = 0;
    service.measurePing(warmupPing);
  }

  std::vector<uint64_t> totalSamples;
  std::vector<uint64_t> serverDecryptSamples;
  std::vector<uint64_t> commSamples;
  std::vector<uint64_t> pingSamples;
  totalSamples.reserve(condition.trials);
  serverDecryptSamples.reserve(condition.trials);
  commSamples.reserve(condition.trials);
  pingSamples.reserve(condition.trials);

  for(int trial = 1; trial <= condition.trials; trial++) {
    // 通信のみの往復時間を、復号と同じ試行の中で計測する
    uint64_t pingMicro = 0;
    service.measurePing(pingMicro);

    DecryptMeasurement measurement;
    bool isSucceeded = service.decrypt(condition.key, condition.encryptedText, condition.iterations,
                                       measurement);

    if(!isSucceeded) {
      result.failureCount++;
    } else {
      if(measurement.plainText != condition.expectedPlainText) {
        result.allPlainTextMatched = false;
      }
      totalSamples.push_back(measurement.totalMicro);
      serverDecryptSamples.push_back(measurement.serverDecryptMicro);
      commSamples.push_back(measurement.commMicro);
      pingSamples.push_back(pingMicro);
    }

    csvWriter.writeRow(trial, result.mode, serverIp, condition.port, condition.encryptedText.size(),
                       condition.iterations, measurement, pingMicro);
  }

  result.total = BenchmarkStatistics::calculate(totalSamples);
  result.serverDecrypt = BenchmarkStatistics::calculate(serverDecryptSamples);
  result.comm = BenchmarkStatistics::calculate(commSamples);
  result.ping = BenchmarkStatistics::calculate(pingSamples);
  return result;
}

void BenchmarkRunner::printSummary(const std::vector<BenchmarkModeResult>& results,
                                   const BenchmarkCondition& condition)
{
  printf("\n=== 復号性能比較 (試行回数=%d, 1リクエストあたりの復号回数=%u) ===\n", condition.trials,
         condition.iterations);
  printf("%-14s %-16s %10s %10s %10s %10s %10s\n", "モード", "項目", "平均", "中央値", "最小",
         "最大", "標準偏差");

  for(const BenchmarkModeResult& result : results) {
    std::string modeName = toDecryptModeName(result.mode);
    printStatisticsRow(modeName + " T_total[us]", result.total);
    printStatisticsRow(modeName + " T_decrypt[us]", result.serverDecrypt);
    printStatisticsRow(modeName + " T_comm[us]", result.comm);
    printStatisticsRow(modeName + " T_ping[us]", result.ping);
    printf("  -> 接続先: %s / 失敗回数: %d / 平文一致: %s\n\n", result.serverIp.c_str(),
           result.failureCount, result.allPlainTextMatched ? "OK" : "NG");
  }
}

void BenchmarkRunner::printStatisticsRow(const std::string& label,
                                         const StatisticsResult& statistics)
{
  printf("%-31s %10.1f %10.1f %10llu %10llu %10.1f\n", label.c_str(), statistics.mean,
         statistics.median, static_cast<unsigned long long>(statistics.min),
         static_cast<unsigned long long>(statistics.max), statistics.standardDeviation);
}
