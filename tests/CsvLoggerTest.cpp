/**
 * @file CsvLoggerTest.cpp
 * @brief CsvLoggerクラスをテストする
 * @author miyahara046
 */

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <string>

#include "CsvLogger.h"


namespace etrobocon2026_test {

// テスト用 CSV ファイルの内容を読み取るユーティリティ関数
static std::string ReadFileContents(const std::string& path)
{
  std::ifstream file(path);

  std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  return contents;
}

// writeHeader() と add() の呼び出し結果が CSV ファイルへ正しく書き込まれることを確認する
TEST(CsvLoggerTest, WriteHeaderAndAddEntriesToFile)
{
  const std::string logPath = "/tests/datafiles/logfiles/runlog.csv";

  // CSV ログを初期化して出力先を設定する
  CsvLogger::init();
  CsvLogger::setFileName(logPath.c_str());

  // ヘッダ行を書き込み、1 件のデータを追加してファイルへ保存する
  CsvLogger::writeHeader();
  CsvLogger::add(42, 100, -100);
  CsvLogger::outputToFile();

  const std::string contents = ReadFileContents("/RasPike-ART/sdk/workspace/etrobocon2026/tests/datafiles/logfiles/runlog.csv");
  EXPECT_EQ(contents, "time,brightness,rightPwm,leftPwm\n42,100,-100\n");
}

// 複数回 add() を呼び出したときに、CSV ファイルに各行が連続して保存されることを確認する
TEST(CsvLoggerTest, MultipleAddCallsAppendLines)
{
  const std::string logPath = "/tests/datafiles/logfiles/runlog.csv";

  // CSV ログを初期化して出力先を設定する
  CsvLogger::init();
  CsvLogger::setFileName(logPath.c_str());

  // ヘッダ行を書き込み、2 件のデータを書き込む
  CsvLogger::writeHeader();
  CsvLogger::add(1, 2, 3);
  CsvLogger::add(4, 5, 6);
  CsvLogger::outputToFile();

  const std::string contents = ReadFileContents("/RasPike-ART/sdk/workspace/etrobocon2026/tests/datafiles/logfiles/runlog.csv");
  EXPECT_EQ(contents, "time,brightness,rightPwm,leftPwm\n1,2,3\n4,5,6\n");
}
}
