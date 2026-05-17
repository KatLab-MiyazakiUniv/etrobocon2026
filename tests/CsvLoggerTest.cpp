#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <string>

#include "CsvLogger.h"

static std::string ReadFileContents(const std::string& path)
{
  std::ifstream file(path);
  EXPECT_TRUE(file.is_open());

  std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  return contents;
}

TEST(CsvLoggerTest, WriteHeaderAndAddEntriesToFile)
{
  const std::string csvDir = "datafiles/logfiles";
  const std::string csvPath = csvDir + "/runlog.csv";

  std::filesystem::create_directories(csvDir);
  CsvLogger::init();
  CsvLogger::writeHeader();
  CsvLogger::add(42, 100, -100);
  CsvLogger::outputToFile();

  const std::string contents = ReadFileContents(csvPath);
  EXPECT_EQ(contents, "time,brightness,rightPwm,leftPwm\n42,100,-100\n");
}

TEST(CsvLoggerTest, MultipleAddCallsAppendLines)
{
  const std::string csvDir = "datafiles/logfiles";
  const std::string csvPath = csvDir + "/runlog.csv";

  std::filesystem::create_directories(csvDir);
  CsvLogger::init();
  CsvLogger::writeHeader();
  CsvLogger::add(1, 2, 3);
  CsvLogger::add(4, 5, 6);
  CsvLogger::outputToFile();

  const std::string contents = ReadFileContents(csvPath);
  EXPECT_EQ(contents, "time,brightness,rightPwm,leftPwm\n1,2,3\n4,5,6\n");
}
