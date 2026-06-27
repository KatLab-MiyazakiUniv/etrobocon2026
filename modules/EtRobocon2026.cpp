/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "Robot.h"
#include "SocketClient.h"
#include "RealNetworkSystem.h"
#include "Course.h"
#include "Odometry.h"
#include "Position.h"
#include ""

void EtRobocon2026::start()
{
  CsvLogger::init();
  CsvLogger::writeHeader();
  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);
  robot.setCourse(Course::Left);
  AreaMaster area1(robot, Area::LineTrace);
  Position pos();
  Odometry odo(pos);

  for(int i = 0; i < 4; i++) {
    odo.update(1000, robot.getIMUControllerInstance().getAzimuth());
    area1.run();
  }
}