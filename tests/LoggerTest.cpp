/**
 * @file CsvLoggerTest.cpp
 * @brief CsvLoggerクラスをテストする
 * @author miyahara046
 */

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <string>

#include "Logger.h"

namespace etrobocon2026_test {

static std::string ReadFileContents(const std::string& path)
{
  std::ifstream file(path);

  std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  return contents;
}

TEST(LoggerTest, InfoWritesInfoLineToFile)
{
  const std::string logPath = "/datafiles/logfiles/logfile.txt";

  Logger::init();
  Logger::setFileName(logPath.c_str());
  Logger::info("test message");
  Logger::outputToFile();

  const std::string contents = ReadFileContents(logPath);
  EXPECT_EQ(contents.find("[INFO] test message\n"), std::string::npos);
}

TEST(LoggerTest, PrintfLogFormatsMessageCorrectly)
{
  const std::string logPath = "/datafiles/logfiles/logfile.txt";


  Logger::init();
  Logger::setFileName(logPath.c_str());
  Logger::printfLog(Logger::DEBUG, "value=%d", 123);
  Logger::outputToFile();

  const std::string contents = ReadFileContents(logPath);
  EXPECT_EQ(contents.find("[DEBUG] value=123\n"), std::string::npos);
}

TEST(LoggerTest, OutputsAllLogLevelsToFile)
{
  const std::string logPath = "/datafiles/logfiles/logfile2.txt";


  Logger::init();
  Logger::setFileName(logPath.c_str());
  Logger::info("info");
  Logger::warning("warning");
  Logger::error("error");
  Logger::debug("debug");
  Logger::outputToFile();

  const std::string contents = ReadFileContents(logPath);
  EXPECT_EQ(contents.find("[INFO] info\n"), std::string::npos);
  EXPECT_EQ(contents.find("[WARNING] warning\n"), std::string::npos);
  EXPECT_EQ(contents.find("[ERROR] error\n"), std::string::npos);
  EXPECT_EQ(contents.find("[DEBUG] debug\n"), std::string::npos);
}
}