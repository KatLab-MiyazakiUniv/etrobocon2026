/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "MotionParser.h"
#include "AreaMaster.h"

void EtRobocon2026::start()
{
  std::cout << "Hello KATLAB" << std::endl;
  Robot robot;
  bool isLeftCourse = true;
  Area area = Area::LineTrace;
  
  MotionParser::createRunCSV(robot, area, isLeftCourse);
  // MotionParser::deleteRunCSV(robot, area, isLeftCourse);
}