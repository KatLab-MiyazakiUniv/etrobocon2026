/**
 * @file   BenchmarkRunner.h
 * @brief  復号モードごとに性能を計測して比較するクラス
 * @author HaruArima08
 */

#ifndef BENCHMARK_RUNNER_H
#define BENCHMARK_RUNNER_H

#include "BenchmarkCsvWriter.h"
#include "BenchmarkStatistics.h"
#include "IDecryptService.h"
#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief ベンチマークの実行条件
 */
struct BenchmarkCondition {
  int trials = 100;               // 計測する試行回数
  int warmupTrials = 10;          // 計測前に捨てる試行回数
  uint32_t iterations = 1;        // 1リクエストあたりの復号回数
  std::string key;                // 復号キー
  std::string encryptedText;      // 暗号文
  std::string expectedPlainText;  // 期待する平文（一致判定に用いる）
  std::string csvFilePath;        // CSVの出力先
  std::string remoteServerIp;     // PC側復号サーバーのIPアドレス
  int port = 0;                   // 復号サーバーのポート番号
};

/**
 * @brief 1モード分の計測結果
 */
struct BenchmarkModeResult {
  DecryptMode mode = DecryptMode::LOCAL_INPROC;  // 復号モード
  std::string serverIp;                          // 接続先のIPアドレス
  bool allPlainTextMatched = true;               // 全試行で平文が期待値と一致したか
  int failureCount = 0;                          // 復号または通信に失敗した回数
  StatisticsResult total;                        // 全体時間の統計量
  StatisticsResult serverDecrypt;                // 演算時間の統計量
  StatisticsResult comm;                         // 通信オーバヘッドの統計量
  StatisticsResult ping;                         // 通信のみの往復時間の統計量
};

class BenchmarkRunner {
 public:
  /**
   * @brief コンストラクタ
   * @param _condition ベンチマークの実行条件
   * @param _csvWriter 計測結果の出力先
   */
  BenchmarkRunner(const BenchmarkCondition& _condition, BenchmarkCsvWriter& _csvWriter);

  /**
   * @brief 1つの復号モードについて計測を行う
   * @param service 計測対象の復号サービス
   * @param serverIp 接続先のIPアドレス（プロセス内復号の場合は"-"）
   * @return BenchmarkModeResult 計測結果
   */
  BenchmarkModeResult run(IDecryptService& service, const std::string& serverIp);

  /**
   * @brief 計測結果を標準出力へ表示する
   * @param results モードごとの計測結果
   * @param condition ベンチマークの実行条件
   */
  static void printSummary(const std::vector<BenchmarkModeResult>& results,
                           const BenchmarkCondition& condition);

 private:
  const BenchmarkCondition& condition;  // ベンチマークの実行条件への参照
  BenchmarkCsvWriter& csvWriter;        // 計測結果の出力先への参照

  /**
   * @brief 統計量を1行で標準出力へ表示する
   * @param label 表示するラベル
   * @param statistics 表示する統計量
   */
  static void printStatisticsRow(const std::string& label, const StatisticsResult& statistics);
};

#endif  // BENCHMARK_RUNNER_H
