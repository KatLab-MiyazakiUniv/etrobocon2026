#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <string>

#include "Logger.h"

static std::string ReadFileContents(const std::string& path)
{
  std::ifstream file(path);
  EXPECT_TRUE(file.is_open());

  std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  return contents;
}

TEST(LoggerTest, InfoWritesInfoLineToFile)
{
  const std::string logDir = "datafiles/logfiles";
  const std::string logPath = logDir + "/logfile.txt";

  std::filesystem::create_directories(logDir);
  Logger::init();
  Logger::info("test message");
  Logger::outputToFile();

  const std::string contents = ReadFileContents(logPath);
  EXPECT_NE(contents.find("[INFO] test message\n"), std::string::npos);
}

TEST(LoggerTest, PrintfLogFormatsMessageCorrectly)
{
  const std::string logDir = "datafiles/logfiles";
  const std::string logPath = logDir + "/logfile.txt";

  std::filesystem::create_directories(logDir);
  Logger::init();
  Logger::printfLog(Logger::DEBUG, "value=%d", 123);
  Logger::outputToFile();

  const std::string contents = ReadFileContents(logPath);
  EXPECT_NE(contents.find("[DEBUG] value=123\n"), std::string::npos);
}

TEST(LoggerTest, OutputsAllLogLevelsToFile)
{
  const std::string logDir = "datafiles/logfiles";
  const std::string logPath = logDir + "/logfile.txt";

  std::filesystem::create_directories(logDir);
  Logger::init();
  Logger::info("info");
  Logger::warning("warning");
  Logger::error("error");
  Logger::debug("debug");
  Logger::outputToFile();

  const std::string contents = ReadFileContents(logPath);
  EXPECT_NE(contents.find("[INFO] info\n"), std::string::npos);
  EXPECT_NE(contents.find("[WARNING] warning\n"), std::string::npos);
  EXPECT_NE(contents.find("[ERROR] error\n"), std::string::npos);
  EXPECT_NE(contents.find("[DEBUG] debug\n"), std::string::npos);
}
