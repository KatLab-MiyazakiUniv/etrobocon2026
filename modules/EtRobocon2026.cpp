/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "RelativeRotation.h"
#include "AbsoluteRotation.h"
#include "RelativeAngleCondition.h"
#include "AbsoluteAngleCondition.h"
#include "Straight.h"
#include "Position.h"
#include "Odometry.h"
#include "DistanceCondition.h"
#include <iostream>
#include <memory>

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  Robot robot;

  Pid::PidGain anglePidGain(0.8, 0.0, 0.05);

  auto relCondition = std::make_unique<RelativeAngleCondition>(robot, 90.0);
  RelativeRotation relRotation(robot, std::move(relCondition), anglePidGain, 90.0);

  Pid::PidGain rightPidGain(0.016, 0.005, 0.0015);
  Pid::PidGain leftPidGain(0.016, 0.005, 0.0015);
  Pid::PidGain aPidGain(0, 0, 0);

  auto Condition = std::make_unique<DistanceCondition>(robot, 100);
  Straight straight(robot, std::move(Condition), 150, rightPidGain, leftPidGain, aPidGain, false);

  Position pos;
  Odometry odometry(pos);

  for(int i = 0; i < 5; i++) {
    straight.run();
    relRotation.run();
    odometry.update(100, robot.getIMUControllerInstance().getAzimuth());
    Logger::info((std::to_string(i) + "回目").c_str());
    Logger::info((std::to_string(pos.getX())).c_str());
    Logger::info((std::to_string(pos.getX())).c_str());
  }
}
