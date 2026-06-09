/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "Robot.h"
#include <iostream>

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  Robot robot;
  int Distance = robot.getUltrasonicSensorInstance().getDistance();

  std::cout << "Distance: " << Distance << std::endl;

}
