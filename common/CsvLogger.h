/**
 * @file CsvLogger.h
 * @brief 走行中の各種値可視化のためのcsvファイル作成に関するクラス
 * @author miyahara046
 */

#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <stdio.h>
#include <filesystem>
#include <string>
#include <fstream>
#include <iostream>
#include <optional>
#include <vector>
#include "ClockUtil.h"
#include "Logger.h"

// デフォルトのログファイルパスとファイル名
#define DEFAULT_CSV_LOG_FILE_PATH "etrobocon2026/datafiles/logfiles/"
#define DEFAULT_CSV_LOG_FILE_NAME "runlog.csv"

/**
 * @struct LogData
 * @brief CsvLoggerで記録する走行データをまとめた構造体
 */
struct LogData {
  std::optional<int> time;           // 走行経過時間
  std::optional<std::string> id;     // 実行しているコマンドのID(コマンド名:ID)
  std::optional<int> brightness;     // 輝度値
  std::optional<int> rightPower;     // 右Power値
  std::optional<int> leftPower;      // 左Power値
  std::optional<double> rightSpeed;  // 右Speed値
  std::optional<double> leftSpeed;   // 左Speed値
  // 今後新しいセンサー値（例: gyro）を追加したい場合は、ここに std::optional<int> gyro;
};

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
   * @param data 走行データ構造体
   */
  static void add(const LogData& data);

  /**
   * @brief 記録した走行ログをファイル出力する
   */
  static void outputToFile();

  /**
   * @brief ログファイルの出力先を変更
   * @param name 出力するファイル名
   * @param path 出力先パス
   */
  static void setFileName(const std::string& name = DEFAULT_CSV_LOG_FILE_NAME,
                          const std::string& path = DEFAULT_CSV_LOG_FILE_PATH);

 private:
  static constexpr int LOG_BUFFER_SIZE = 65536;  // ログ全体のバッファサイズ
  static char logs[LOG_BUFFER_SIZE];             // ログのバッファ
  static int currentIndex;                       // ログの現在のインデックス

  // 現在使用しているログファイルパス
  static std::string fileName;

  // ヘッダー定義を管理する配列
  static const std::vector<std::string> HEADERS;

  CsvLogger();  // インスタンス化禁止
};

#endif