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

  // テスト用ログファイルの内容を読み取る関数
  static std::string ReadFileContents(const std::string& path)
  {
    std::ifstream file(path);
    std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return contents;
  }

  // テスト実行前に古いファイルを消す関数
  static void CleanUpTestFile(const std::string& fullPath)
  {
    if (std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
  }

  // info() で出力したログがファイルに保存されることを確認する
  TEST(LoggerTest, InfoWritesInfoLineToFile)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string fileName = "logfile.txt";

    CleanUpTestFile(logPath + fileName);

    // ログを初期化
    Logger::init();

    Logger::setFileName(fileName, logPath);
    // INFO レベルのメッセージをログバッファに追加し、ファイルへ保存する
    Logger::info("test message");
    Logger::outputToFile();

    const std::string contents = ReadFileContents(logPath + fileName);
    EXPECT_NE(contents.find("[INFO] test message\n"), std::string::npos);
  }

  // printfLog() がフォーマット済みメッセージを正しく書き出すことを確認する
  TEST(LoggerTest, PrintfLogFormatsMessageCorrectly)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string fileName = "logfile1.txt";

    CleanUpTestFile(logPath + fileName);

    // ログを初期化して出力先を設定する
    Logger::init();
    Logger::setFileName(fileName, logPath);

    // DEBUG メッセージをフォーマット出力する
    Logger::printfLog(Logger::DEBUG, "value=%d", 123);
    Logger::outputToFile();

    const std::string contents = ReadFileContents(logPath + fileName);
    EXPECT_NE(contents.find("[DEBUG] value=123\n"), std::string::npos);
  }

  // 4 種類のログレベルがそれぞれファイルに出力されることを確認する
  TEST(LoggerTest, OutputsAllLogLevelsToFile)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string fileName = "logfile2.txt";

    CleanUpTestFile(logPath + fileName);

    // ログを初期化して、出力先ファイル名を設定する
    Logger::init();
    Logger::setFileName(fileName, logPath);

    // 4 種類のログレベルを順に出力する
    Logger::info("info");
    Logger::warning("warning");
    Logger::error("error");
    Logger::debug("debug");
    Logger::outputToFile();

    // 各出力がファイルに書き込まれていることを確認する
    const std::string contents = ReadFileContents(logPath + fileName);
    EXPECT_NE(contents.find("[INFO] info\n"), std::string::npos);
    EXPECT_NE(contents.find("[WARNING] warning\n"), std::string::npos);
    EXPECT_NE(contents.find("[ERROR] error\n"), std::string::npos);
    EXPECT_NE(contents.find("[DEBUG] debug\n"), std::string::npos);
  }

  // 異常なファイルパスでのエラーハンドリング
  TEST(LoggerTest, OutputToFileHandlesInvalidPathGracefully)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string fileName = "logfileerror.txt";
    Logger::init();
    Logger::setFileName(fileName, logPath);

    // 出力処理が例外を投げずに安全にリターンするかを検証
    Logger::setFileName("invalid_file_???", "/invalid_dir_***/");

    Logger::info("Test error handling");

    // 出力先が無効な場合でも、例外が発生せずに処理が完了することを確認
    EXPECT_NO_THROW({ Logger::outputToFile(); });
  }

  // LOG_CREATE / LOG_DESTROY マクロの展開結果がファイルに正しく出力されることを確認する
  TEST(LoggerTest, MacrosWriteExpectedLogs)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string fileName = "macrotestlog.txt";

    CleanUpTestFile(logPath + fileName);

    Logger::init();
    Logger::setFileName(fileName, logPath);

    // マクロを呼び出す
    LOG_CREATE();
    LOG_DESTROY();

    Logger::outputToFile();
    const std::string contents = ReadFileContents(logPath + fileName);

    // マクロが展開され、関数名(TestBodyなど)や "created", "destroyed" が含まれているか検証
    EXPECT_NE(contents.find("created"), std::string::npos);
    EXPECT_NE(contents.find("destroyed"), std::string::npos);
  }

}  // namespace etrobocon2026_test