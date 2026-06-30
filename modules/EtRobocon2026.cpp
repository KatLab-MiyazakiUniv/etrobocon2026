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
#include "RelativeRotation.h"
#include "RelativeAngleCondition.h"
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

  Pid::PidGain rotationPidGain(0.9, 0.7, 0.1);
  Pid::PidGain rightPid(0.016, 0.005, 0.0015);
  Pid::PidGain leftPid(0.016, 0.0045, 0.0015);
  Pid::PidGain anglePid(0.036, 0.012, 0.03);

  Goal goal[] = { { 500.0, 0.0 }, { 500.0, 500.0 }, { 0.0, 500.0 }, { 0.0, 0.0 } };

  robot.getIMUControllerInstance().resetAzimuth();
  for(int j = 0; j < 10; j++) {
    for(int i = 0; i < 4; i++) {
      double calHead = nav.calculateHeading(goal[i].x, goal[i].y);
      double calDis = nav.calculateDistance(goal[i].x, goal[i].y);

      auto absCondition = std::make_unique<RelativeAngleCondition>(robot, calHead);
      RelativeRotation absRotaion(robot, std::move(absCondition), rotationPidGain, calHead);
      absRotaion.run();

      ClockUtil::wait(1000);

      Straight straight(robot, std::make_unique<DistanceCondition>(robot, calDis), 300, rightPid,
                        leftPid, anglePid, true);
      straight.run();

      ClockUtil::wait(1000);

      int32_t left = robot.getWheelMotorControllerInstance().getLeftCount();
      int32_t right = robot.getWheelMotorControllerInstance().getRightCount();

      odo.update(left, right, robot.getIMUControllerInstance().getAzimuth());

      std::cout << "num = " << pos.getX() << "\n";
      std::cout << "num = " << pos.getY() << "\n";
      Logger::printfLog(Logger::INFO, "x = %lf", pos.getX());
      Logger::printfLog(Logger::INFO, "y = %lf", pos.getY());
    }
  }
}