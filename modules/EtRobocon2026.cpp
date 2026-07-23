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
#include "RealNetworkSystem.h"
#include "BatteryController.h"

void EtRobocon2026::start()
{
  CsvLogger::init();
  CsvLogger::writeHeader();
  Logger::info("Hello KATLAB");
  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);
  int voltage = BatteryController::getVoltage();
  Logger::printfLog(Logger::INFO, "バッテリー電圧: %d mV", voltage);
  robot.getCameraSocketClientInstance().connectToServer();
  robot.setCourse(Course::Left);
  // robot.setEdge(Edge::LeftEdge);
  robot.setEdge(Edge::RightEdge);
  // Area lineTraceArea = Area::LineTrace;
  // lineTraceAreaMaster.run();

  if(robot.getIndexOfLabel() == 2) {
    Area yellowArea = Area::Yellow;
    AreaMaster carryAreaMaster(robot, yellowArea);
    carryAreaMaster.run();
  } else if(robot.getIndexOfLabel() == 1) {
    Area blueArea = Area::Blue;
    AreaMaster carryAreaMaster(robot, blueArea);
    carryAreaMaster.run();

  } else {
    Area redArea = Area::Red;
    AreaMaster carryAreaMaster(robot, redArea);
    carryAreaMaster.run();
  }

  CsvLogger::outputToFile();
}