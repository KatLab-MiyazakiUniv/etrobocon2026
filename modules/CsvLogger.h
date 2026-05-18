/**
 * @file CsvLogger.h
 * @brief 走行中の各種値可視化のためのcsvファイル作成に関するクラス
 * @author miyahara046
 */

#ifndef CSV_LOGGER_HPP
#define CSV_LOGGER_HPP

#include <stdio.h>
#include <stdint.h>
#include <filesystem>
#include <string.h>
// #include "ClockUtil.h"

class CsvLogger {
 public:
  /**
   * @brief CSVログの初期化
   */
  static void init();

  /**
   * @brief CSVヘッダを書き込む
   */
  static void writeHeader();

  /**
   * @brief 各種値を追加する
   * @param brightness 輝度値
   * @param rightPwm 右Power値
   * @param leftPwm 左Power値
   */
  static void add(int brightness, int rightPwm, int leftPwm);

  /**
   * @brief 記録した走行ログをファイル出力する
   */
  static void outputToFile();

  /**
   * @brief ログファイルの出力先を変更
   * @param path 出力先パス
   */
  static void setFileName(const char* path);

 private:
  static constexpr int LOG_BUFFER_SIZE = 65536;  // ログ全体のバッファサイズ
  static constexpr int LINE_BUFFER_SIZE = 128;   // 1メッセージのバッファサイズ
  static char logs[LOG_BUFFER_SIZE];             // ログのバッファ
  static int currentIndex;                       // ログの現在のインデックス

  // デフォルトのログファイルパス
  static constexpr const char* DEFAULT_CSV_NAME
      = "/RasPike-ART/sdk/workspace/etrobocon2026/datafiles/logfiles/runlog.csv";

  // 現在使用しているログファイルパス
  static std::string fileName;
};

#endif