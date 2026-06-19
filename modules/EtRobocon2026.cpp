/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "Robot.h"
#include "SocketClient.h"
#include "RealNetworkSystem.h"
#include "LineTrace.h"
#include "SensorColorCondition.h"
#include "RunningTimeCondition.h"
#include "DistanceCondition.h"

void EtRobocon2026::start()
{
  CsvLogger::init();
  CsvLogger::writeHeader();
  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);
  double targetSpeed = 300;
  int taergetBrightness;

  double targetDistance = 2000;
  LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed, taergetBrightness, Pid::PidGain(0.0, 0.0, 0.0));
  lineTrace.run();
  CsvLogger::outputToFile();
}
