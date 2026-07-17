/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "GoalNavigation.h"
#include "GoalDistanceCondition.h"
#include "Pid.h"
#include "Robot.h"

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  Robot robot;

  struct Goal {
    double x;
    double y;
  };

  Pid anglePidGain = {}

  Goal goal[] = { { 1000.0, 0.0 }, { 1000.0, 500.0 }, { 250.0, 500.0 }, { 250.0, 1250.0 } };

  GoalNavigation navigation(
      robot,
      std::make_unique<GoalDistanceCondition>(robot, goal[0].x, goal[0].y,
                                              10.0),  // 目標から10mm以内で終了
      goal[0].x, goal[0].y, 300, anglePidGain);

  navigation.run();
}