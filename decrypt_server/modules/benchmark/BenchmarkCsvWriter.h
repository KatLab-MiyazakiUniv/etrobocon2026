/**
 * @file   BenchmarkCsvWriter.h
 * @brief  復号性能の計測結果をCSVへ出力するクラス
 * @author HaruArima08
 */

#ifndef BENCHMARK_CSV_WRITER_H
#define BENCHMARK_CSV_WRITER_H

#include "DecryptMode.h"
#include "IDecryptService.h"
#include <cstdint>
#include <fstream>
#include <string>

class BenchmarkCsvWriter {
 public:
  /**
   * @brief コンストラクタ
   * @param _filePath 出力先のCSVファイルパス
   */
  explicit BenchmarkCsvWriter(const std::string& _filePath);

  /**
   * @brief デストラクタ
   */
  ~BenchmarkCsvWriter();

  /**
   * @brief CSVファイルを開き、ヘッダ行を書き込む
   * @return true 書き込み準備に成功した場合
   * @return false ファイルを開けなかった場合
   */
  bool open();

  /**
   * @brief 1試行分の計測結果を1行書き込む
   * @param trial 試行番号
   * @param mode 復号モード
   * @param serverIp 接続先のIPアドレス
   * @param port 接続先のポート番号
   * @param encryptedTextLength 暗号文のバイト長
   * @param iterations 1リクエストあたりの復号回数
   * @param measurement 計測結果
   * @param pingMicro 同条件で計測した通信のみの往復時間(マイクロ秒)
   */
  void writeRow(int trial, DecryptMode mode, const std::string& serverIp, int port,
                size_t encryptedTextLength, uint32_t iterations,
                const DecryptMeasurement& measurement, uint64_t pingMicro);

  /**
   * @brief CSVファイルを閉じる
   */
  void close();

 private:
  std::string filePath;  // 出力先のCSVファイルパス
  std::ofstream ofs;     // 出力ストリーム
};

#endif  // BENCHMARK_CSV_WRITER_H
