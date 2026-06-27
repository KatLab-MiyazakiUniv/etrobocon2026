/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "Robot.h"
#include "SocketClient.h"
#include "RealNetworkSystem.h"
#include "AreaMaster.h"
#include "Course.h"
#include "Odometry.h"
#include "Position.h"
#include "Navigator.h"
#include "Straight.h"
#include "AbsoluteRotation.h"
#include "AbsoluteAngleCondition.h"
#include "DistanceCondition.h"
#include "Mileage.h"

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);
  Position pos;
  Odometry odo(pos);
  Navigator nav(pos);

  struct Goal {
    double x;
    double y;
  };

  Goal goal[] = { { 500.0, 0.0 }, { 500.0, 500.0 }, { 0.0, 500.0 }, { 0.0, 0.0 } };

  robot.getIMUControllerInstance().resetAzimuth();

  for(int i = 0; i < 4; i++) {
    double calHead = nav.calculateHeading(goal[i]);
    double calDis = nav.calculateDistance(goal[i]);

    odo.update(500, robot.getIMUControllerInstance().getAzimuth());
    std::cout << "num = " << pos.getX() << "\n";
    std::cout << "num = " << pos.getY() << "\n";
    Logger::printfLog(Logger::Level info, "x = %lf", pos.getX());
    Logger::printfLog(Logger::Level info, "y = %lf", pos.getY());
  }
}