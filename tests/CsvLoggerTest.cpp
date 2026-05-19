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

using namespace std;

namespace etrobocon2026_test {

  // テスト用 CSV ファイルの内容を読み取るユーティリティ関数
  static std::string ReadFileContents(const std::string& path)
  {
    std::ifstream file(path);
    if (!file.is_open()) return "";
    std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return contents;
  }

  // テスト実行前に過去のテストファイルを削除するユーティリティ関数
  static void CleanUpTestFile(const std::string& fullPath)
  {
    if (std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
  }

  // writeHeader() と add() の呼び出し結果が CSV ファイルへ正しく書き込まれることを確認する
  TEST(CsvLoggerTest, WriteHeaderAndAddEntriesToFile)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string logFileName = "runlog.csv";
    const std::string fullPath = logPath + logFileName;

    // 前回のテストデータが残っていたら削除する
    CleanUpTestFile(fullPath);

    // CSV ログを初期化して出力先を設定する
    CsvLogger::init();
    CsvLogger::setFileName(logFileName, logPath);

    // ヘッダ行を書き込む
    CsvLogger::writeHeader();

    // 構造体に値をセットしてデータを追加する
    LogData data;
    data.time = 0; // コメントアウトされていたtimeに対応できるように定義
    data.brightness = 42;
    data.rightPower = 100;
    data.leftPower = -100;
    CsvLogger::add(data);

    // ファイルへ保存する
    CsvLogger::outputToFile();

    const std::string contents = ReadFileContents(fullPath);
    EXPECT_NE(contents.find("time,brightness,rightPower,leftPower\n0,42,100,-100\n"),
              std::string::npos);
  }

  // 複数回 add() を呼び出したときに、CSV ファイルに各行が連続して保存されることを確認する
  TEST(CsvLoggerTest, MultipleAddCallsAppendLines)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string logFileName = "runlog2.csv";
    const std::string fullPath = logPath + logFileName;

    CleanUpTestFile(fullPath);

    CsvLogger::init();
    CsvLogger::setFileName(logFileName, logPath);

    CsvLogger::writeHeader();

    // 1件目のデータ
    LogData data1{1, 2, 3, 4};
    CsvLogger::add(data1);

    // 2件目のデータ
    LogData data2{5, 6, 7, 8};
    CsvLogger::add(data2);

    CsvLogger::outputToFile();

    const std::string contents = ReadFileContents(fullPath);
    EXPECT_NE(contents.find("time,brightness,rightPower,leftPower\n1,2,3,4\n5,6,7,8\n"),
              std::string::npos);
  }

  // 一部の値が欠けているときに空文字（,,）でカンマが維持されるか確認する
  TEST(CsvLoggerTest, OptionalValuesRenderEmptyFields)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string logFileName = "runlog_optional.csv";
    const std::string fullPath = logPath + logFileName;

    CleanUpTestFile(fullPath);

    CsvLogger::init();
    CsvLogger::setFileName(logFileName, logPath);
    CsvLogger::writeHeader();

    // 輝度(brightness)と左パワー(leftPower)だけ値があり、他が欠けている場合
    LogData partialData;
    partialData.brightness = 60;
    partialData.leftPower = 35;
    // time と rightPower は std::nullopt のまま

    CsvLogger::add(partialData);
    CsvLogger::outputToFile();

    const std::string contents = ReadFileContents(fullPath);
    // 期待される出力: "time,brightness,rightPower,leftPower\n,60,,35\n"
    EXPECT_NE(contents.find("time,brightness,rightPower,leftPower\n,60,,35\n"),
              std::string::npos);
  }
}  // namespace etrobocon2026_test