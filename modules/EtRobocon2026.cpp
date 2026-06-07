/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "LineTrace.h"
#include "DistanceCondition.h"

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");
  Logger::info("LineTrace test start");

  Robot robot;

  // PID（適当でもOK）
  Pid::PidGain rightPid = { 0.00535, 0.00115, 0.0 };
  Pid::PidGain leftPid = { 0.00578, 0.00085, 0.0 };
  Pid::PidGain tracePid = { 0.0055, 0.0009, 0.0 };

  double targetSpeed = 100.0;
  int targetBrightness = 50;
  double targetDistance = 10.0;

  LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed, targetBrightness, rightPid, leftPid, tracePid);

  lineTrace.run();

  Logger::info("LineTrace finished");
}
