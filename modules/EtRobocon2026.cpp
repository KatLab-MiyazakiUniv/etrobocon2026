/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "Logger.h"
#include "CsvLogger.h"

void EtRobocon2026::start()
{
  std::cout << "Hello KATLAB" << std::endl;
  Logger::init();
  Logger::info("Start logging");
  Logger::warning("This is a warning");
  Logger::error("This is an error");
  Logger::debug("Debug message");
  Logger::printfLog(Logger::INFO, "Formatted value=%d", 2026);
  Logger::outputToFile();

  CsvLogger::init();
  CsvLogger::writeHeader();
  CsvLogger::add(123, 50, -50);
  CsvLogger::add(200, 75, -75);
  CsvLogger::outputToFile();
}
