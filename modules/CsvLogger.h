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
   * @param rightPower 右Power値
   * @param leftPower 左Power値
   */
  static void add(int brightness, int rightPower, int leftPower);

  /**
   * @brief 記録した走行ログをファイル出力する
   */
  static void outputToFile();

  /**
   * @brief ログファイルの出力先を変更
   * @param path 出力先パス
   */
  static void setFileName(const std::string& path);

 private:
  static constexpr int LOG_BUFFER_SIZE = 65536;  // ログ全体のバッファサイズ
  static char logs[LOG_BUFFER_SIZE];             // ログのバッファ
  static int currentIndex;                       // ログの現在のインデックス

  // 現在使用しているログファイルパス
  static std::string fileName;

  CsvLogge(); // インスタンス化を禁止する
};

#endif