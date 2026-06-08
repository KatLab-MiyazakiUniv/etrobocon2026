/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "Robot.h"
#include "LineTrace.h"
#include "DistanceCondition.h"

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  Robot robot;
  robot.setCourse(Course::Left);
  robot.setEdge(Edge::LeftEdge);

  DistanceCondition distanceCondition(robot, 100.0);

  LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, 400.0), 400.0, 50,
                      Pid::PidGain{ 0.0055, 0.0009, 0.0 }, Pid::PidGain{ 0.00535, 0.00115, 0.000 },
                      Pid::PidGain{ 0.00578, 0.0008535, 0.000 });

  lineTrace.run();
}