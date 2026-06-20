/**
 * @file FastCsvLogger.h
 * @brief 走行中の各種値可視化のためのcsvファイル作成に関するクラス（高速版）
 */

#ifndef FAST_CSV_LOGGER_H
#define FAST_CSV_LOGGER_H

#include <stdio.h>
#include <filesystem>
#include <string>
#include <fstream>
#include <iostream>
#include <optional>
#include <vector>
#include "ClockUtil.h"
#include "Logger.h"
#include "CsvLogger.h" // LogData 構造体を使用するため

// デフォルトのログファイルパスとファイル名
#define DEFAULT_FAST_CSV_LOG_FILE_PATH "etrobocon2026/datafiles/logfiles/"
#define DEFAULT_FAST_CSV_LOG_FILE_NAME "fast_runlog.csv"

class FastCsvLogger {
 public:
  /**
   * @brief CSVログの初期化
   */
  static void init();

  /**
   * @brief CSVヘッダを出力対象に設定する
   */
  static void writeHeader();

  /**
   * @brief 各種値を追加する（走行中はメモリへのバッファリングのみで高速）
   * @param data 走行データ構造体
   */
  static void add(const LogData& data);

  /**
   * @brief 記録した走行ログをファイル出力する（ここで文字列変換とファイルI/Oを行う）
   */
  static void outputToFile();

  /**
   * @brief ログファイルの出力先を変更
   * @param name 出力するファイル名
   * @param path 出力先パス
   */
  static void setFileName(const std::string& name = DEFAULT_FAST_CSV_LOG_FILE_NAME,
                          const std::string& path = DEFAULT_FAST_CSV_LOG_FILE_PATH);

 private:
  static constexpr size_t MAX_LOG_SIZE = 10000;      // 最大ログ記録件数
  static std::vector<LogData> logBuffer;             // ログのメモリバッファ
  static bool hasHeader;                             // ヘッダー書き込みフラグ

  // 現在使用しているログファイルパス
  static std::string fileName;

  // ヘッダー定義を管理する配列
  static const std::vector<std::string> HEADERS;

  FastCsvLogger();  // インスタンス化禁止
};

#endif
