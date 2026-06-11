/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "AreaMaster.h"
#include "Robot.h"
#include "Course.h"
#include "CsvLogger.h"

void EtRobocon2026::start()
{
  CsvLogger::init();
  CsvLogger::writeHeader();

  Logger::info("Hello KATLAB");
  Robot robot;
  robot.setCourse(Course::Left);
  Area lineTraceArea = Area::LineTrace;
  AreaMaster lineTraceAreaMaster(robot, lineTraceArea);
  lineTraceAreaMaster.run();
  CsvLogger::outputToFile();
  
}
