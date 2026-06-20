/**
 * @file FastCsvLoggerTest.cpp
 * @brief FastCsvLoggerクラスをテストする
 */

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <string>

#include "FastCsvLogger.h"

using namespace std;

namespace etrobocon2026_test {

  // テスト用 CSV ファイルの内容を読み取るユーティリティ関数
  static std::string ReadFileContents(const std::string& path)
  {
    std::ifstream file(path);
    if(!file.is_open()) return "";
    std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return contents;
  }

  // テスト実行前に過去のテストファイルを削除するユーティリティ関数
  static void CleanUpTestFile(const std::string& fullPath)
  {
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
  }

  // writeHeader() と add() の呼び出し結果が CSV ファイルへ正しく書き込まれることを確認する
  TEST(FastCsvLoggerTest, WriteHeaderAndAddEntriesToFile)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string logFileName = "fast_runlog.csv";
    const std::string fullPath = logPath + logFileName;

    // 前回のテストデータが残っていたら削除する
    CleanUpTestFile(fullPath);

    // CSV ログを初期化して出力先を設定する
    FastCsvLogger::init();
    FastCsvLogger::setFileName(logFileName, logPath);

    // ヘッダ行を設定する
    FastCsvLogger::writeHeader();

    // 構造体に値をセットしてデータを追加する
    LogData data;
    data.time = 42;
    data.id = "CDL";
    data.brightness = 100;
    data.rightPower = 100;
    data.leftPower = -100;
    data.rightSpeed = 50.53;
    data.leftSpeed = -50.50;
    FastCsvLogger::add(data);

    // ファイルへ保存する
    FastCsvLogger::outputToFile();

    const std::string contents = ReadFileContents(fullPath);
    EXPECT_NE(contents.find("time,comand:id,brightness,rightPower,leftPower,rightSpeed,leftSpeed"),
              std::string::npos);
    EXPECT_NE(contents.find("42,CDL,100,100,-100,50.53,-50.5"), std::string::npos);
  }

  // 複数回 add() を呼び出したときに、CSV ファイルに各行が連続して保存されることを確認する
  TEST(FastCsvLoggerTest, MultipleAddCallsAppendLines)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string logFileName = "fast_runlog2.csv";
    const std::string fullPath = logPath + logFileName;

    CleanUpTestFile(fullPath);

    FastCsvLogger::init();
    FastCsvLogger::setFileName(logFileName, logPath);

    FastCsvLogger::writeHeader();

    // 1件目のデータ
    LogData data1{ 1, "CDL", 2, 3, 4, 5.5, 6.5 };
    FastCsvLogger::add(data1);

    // 2件目のデータ
    LogData data2{ 7, "CDL", 8, 9, 10, 11.123, 12.0 };
    FastCsvLogger::add(data2);

    FastCsvLogger::outputToFile();

    const std::string contents = ReadFileContents(fullPath);
    EXPECT_NE(contents.find("time,comand:id,brightness,rightPower,leftPower,rightSpeed,leftSpeed"),
              std::string::npos);
    EXPECT_NE(contents.find("1,CDL,2,3,4,5.5,6.5"), std::string::npos);
    EXPECT_NE(contents.find("7,CDL,8,9,10,11.123,12"), std::string::npos);
  }

  // 一部の値が欠けているときに空文字（,,）でカンマが維持されるか確認する
  TEST(FastCsvLoggerTest, OptionalValuesRenderEmptyFields)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string logFileName = "fast_runlog_optional.csv";
    const std::string fullPath = logPath + logFileName;

    CleanUpTestFile(fullPath);

    FastCsvLogger::init();
    FastCsvLogger::setFileName(logFileName, logPath);
    FastCsvLogger::writeHeader();

    // 時間（time）と輝度(brightness)、左パワー(leftPower)だけ値があり、他が欠けている場合
    LogData partialData;
    partialData.time = 90;
    partialData.brightness = 60;
    partialData.leftPower = 35;

    FastCsvLogger::add(partialData);
    FastCsvLogger::outputToFile();

    const std::string contents = ReadFileContents(fullPath);
    EXPECT_NE(contents.find("time,comand:id,brightness,rightPower,leftPower,rightSpeed,leftSpeed"),
              std::string::npos);
    EXPECT_NE(contents.find("90,,60,,35,,"), std::string::npos);
  }
}  // namespace etrobocon2026_test
