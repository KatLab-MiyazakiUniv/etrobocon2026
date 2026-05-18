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

static std::string ReadFileContents(const std::string& path)
{
  std::ifstream file(path);

  std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  return contents;
}

TEST(CsvLoggerTest, WriteHeaderAndAddEntriesToFile)
{
  const std::string csvDir = "/RasPike-ART/sdk/workspace/etrobocon2026/datafiles/logfiles";
  const std::string csvPath = csvDir + "/runlog.csv";

  CsvLogger::init();
  CsvLogger::writeHeader();
  CsvLogger::add(42, 100, -100);
  CsvLogger::outputToFile();

  const std::string contents = ReadFileContents(csvPath);
  EXPECT_EQ(contents, "time,brightness,rightPwm,leftPwm\n42,100,-100\n");
}

TEST(CsvLoggerTest, MultipleAddCallsAppendLines)
{
  const std::string csvDir = "/RasPike-ART/sdk/workspace/etrobocon2026/datafiles/logfiles/";
  const std::string csvPath = csvDir + "/runlog.csv";

  CsvLogger::init();
  CsvLogger::writeHeader();
  CsvLogger::add(1, 2, 3);
  CsvLogger::add(4, 5, 6);
  CsvLogger::outputToFile();

  const std::string contents = ReadFileContents(csvPath);
  EXPECT_EQ(contents, "time,brightness,rightPwm,leftPwm\n1,2,3\n4,5,6\n");
}
}
