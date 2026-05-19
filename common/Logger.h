/**
 * @file Logger.h
 * @brief 走行中のログを出力、保存するクラス
 * @author miyahara046
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <filesystem>
#include <stdarg.h>
#include <string>
#include <fstream>
#include <iostream>

// #include"ClockUtil.h"

// デフォルトのログファイルパスとファイル名
#define DEFAULT_LOG_FILE_PATH "etrobocon2026/datafiles/logfiles/"
#define DEFAULT_LOG_FILE_NAME "logfile.txt"

class Logger {
 public:
  /**
   * @brief ログの分類分け用enumクラス
   * @param INFO 通常ログ
   * @param WARNING 警告ログ
   * @param ERROR エラーログ
   * @param DEBUG デバッグログ
   */
  enum Level { INFO, WARNING, ERROR, DEBUG };

  /**
   * @brief ログの初期化
   */
  static void init();

  /**
   * @brief 通常ログの出力
   * @param message 出力するメッセージ
   */
  static void info(const char* message);

  /**
   * @brief Warningログの出力
   * @param message 出力するメッセージ
   */
  static void warning(const char* message);

  /**
   * @brief Errorログの出力
   * @param message 出力するメッセージ
   */
  static void error(const char* message);

  /**
   * @brief Debugログの出力
   * @param message 出力するメッセージ
   */
  static void debug(const char* message);

  /**
   * @brief フォーマットされたログの出力
   * @param level ログレベル
   * @param format printf形式のフォーマット文字列
   */
  static void printfLog(Level level, const char* format, ...);

  /**
   * @brief ログをファイル出力
   */
  static void outputToFile();

  /**
   * @brief ログファイルの出力先を変更
   * @param name 出力するファイル名
   * @param path 出力先パス
   */
  static void setFileName(const std::string& name = DEFAULT_LOG_FILE_NAME,
                          const std::string& path = DEFAULT_LOG_FILE_PATH);

 private:
  static constexpr int LOG_BUFFER_SIZE = 65536;    // ログ全体のバッファサイズ
  static constexpr int MESSAGE_BUFFER_SIZE = 256;  // 1メッセージのバッファサイズ

  static char logs[LOG_BUFFER_SIZE];  // ログのバッファ
  static int currentIndex;            // ログの現在のインデックス

  // 現在使用しているログファイルパス
  static std::string fileName;

  /**
   * @brief 出力するログを整形して保存する
   * @param level ログレベル
   * @param message 出力するメッセージ
   */
  static void write(Level level, const char* message);

  /**
   * @brief ログレベル文字列変換
   * @param level ログレベル
   */
  static const char* levelToString(Level level);

  /**
   * @brief ANSIカラーコードへの変換
   * @param level ログレベル
   */
  static const char* levelToColor(Level level);

  Logger();  // インスタンス化禁止
};

// インスタンスの生死を記録
#define LOG_CREATE() Logger::printfLog(Logger::INFO, "%s created", __PRETTY_FUNCTION__)

#define LOG_DESTROY() Logger::printfLog(Logger::INFO, "%s destroyed", __PRETTY_FUNCTION__)

#endif