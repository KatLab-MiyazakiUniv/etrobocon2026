/**
 * @file Logger.h
 * @brief 走行中のログを出力、保存するクラス
 * @author miyahara046
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
// #include"ClockUtil.h"
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

 private:
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

  static constexpr int LOG_BUFFER_SIZE = 65536;    // ログ全体のバッファサイズ
  static constexpr int MESSAGE_BUFFER_SIZE = 256;  // 1メッセージのバッファサイズ

  inline static char logs[LOG_BUFFER_SIZE] = {};  // ログ全体のバッファ
  inline static int currentIndex = 0;             // 現在のログの末尾のインデックス

  static constexpr const char* fileName = "datafiles/logfiles/logfile.txt";  // ログファイルのパス
};

#endif