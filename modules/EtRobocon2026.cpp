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
#include "SocketClient.h"
#include "RealNetworkSystem.h"

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);

  struct Goal {
    double x;
    double y;
  };

  Pid::PidGain anglePid(0.036, 0.012, 0.03);
  Pid::PidGain rotationPidGain(0.9, 0.7, 0.1);
  Pid::PidGain rightPid(0.016, 0.005, 0.0015);
  Pid::PidGain leftPid(0.016, 0.0045, 0.0015);

  Goal goal[] = { { 1000.0, 0.0 }, { 1000.0, 500.0 }, { 250.0, 500.0 }, { 250.0, 1250.0 } };

  GoalNavigation navigation(
      robot,
      std::make_unique<GoalDistanceCondition>(robot, goal[0].x, goal[0].y,
                                              10.0),  // 目標から10mm以内で終了
      goal[0].x, goal[0].y, 300, rightPid, leftPid, anglePid);

  navigation.run();
}