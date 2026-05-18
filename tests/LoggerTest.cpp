/**
 * @file LoggerTest.cpp
 * @brief Loggerクラスをテストする
 * @author miyahara046
 */

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <string>

#include "Logger.h"

namespace etrobocon2026_test {

  // テスト用ログファイルの内容を読み取るユーティリティ関数
  static std::string ReadFileContents(const std::string& path)
  {
    std::ifstream file(path);
    std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return contents;
  }

  // info() で出力したログがファイルに保存されることを確認する
  TEST(LoggerTest, InfoWritesInfoLineToFile)
  {
    const std::string logPath = "/tests/datafiles/logfiles/logfile.txt";
    const std::string expectedPath
        = "/RasPike-ART/sdk/workspace/etrobocon2026/tests/datafiles/logfiles/logfile.txt";

    // ログを初期化して、出力先ファイル名を設定する
    Logger::init();
    Logger::setFileName(logPath.c_str());

    // INFO レベルのメッセージをログバッファに追加し、ファイルへ保存する
    Logger::info("test message");
    Logger::outputToFile();

    const std::string contents = ReadFileContents(expectedPath);
    EXPECT_EQ(contents.find("[INFO] test message\n"), std::string::npos);
  }

  // printfLog() がフォーマット済みメッセージを正しく書き出すことを確認する
  TEST(LoggerTest, PrintfLogFormatsMessageCorrectly)
  {
    const std::string logPath = "/tests/datafiles/logfiles/logfile.txt";
    const std::string expectedPath
        = "/RasPike-ART/sdk/workspace/etrobocon2026/tests/datafiles/logfiles/logfile.txt";

    // ログを初期化して出力先を設定する
    Logger::init();
    Logger::setFileName(logPath.c_str());

    // DEBUG メッセージをフォーマット出力する
    Logger::printfLog(Logger::DEBUG, "value=%d", 123);
    Logger::outputToFile();

    const std::string contents = ReadFileContents(expectedPath);
    EXPECT_EQ(contents.find("[DEBUG] value=123\n"), std::string::npos);
  }

  // 4 種類のログレベルがそれぞれファイルに出力されることを確認する
  TEST(LoggerTest, OutputsAllLogLevelsToFile)
  {
    const std::string logPath = "/tests/datafiles/logfiles/logfile2.txt";
    const std::string expectedPath
        = "/RasPike-ART/sdk/workspace/etrobocon2026/tests/datafiles/logfiles/logfile2.txt";

    // ログを初期化して、出力先ファイル名を設定する
    Logger::init();
    Logger::setFileName(logPath.c_str());

    // 4 種類のログレベルを順に出力する
    Logger::info("info");
    Logger::warning("warning");
    Logger::error("error");
    Logger::debug("debug");
    Logger::outputToFile();

    // 各出力がファイルに書き込まれていることを確認する
    const std::string contents = ReadFileContents(expectedPath);
    EXPECT_EQ(contents.find("[INFO] info\n"), std::string::npos);
    EXPECT_EQ(contents.find("[WARNING] warning\n"), std::string::npos);
    EXPECT_EQ(contents.find("[ERROR] error\n"), std::string::npos);
    EXPECT_EQ(contents.find("[DEBUG] debug\n"), std::string::npos);
  }
}  // namespace etrobocon2026_test