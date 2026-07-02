/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "AbsoluteRotation.h"
#include "AbsoluteAngleCondition.h"
#include "RelativeRotation.h"
#include "RelativeAngleCondition.h"
#include "Robot.h"

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);

  Pid::PidGain rotationPidGain(0.9, 0.7, 0.1);
  // 0.036,0.02,0.03

  double calHead = 90.0;

  auto absCondition = std::make_unique<AbsoluteAngleCondition>(robot, calHead, 0.0);
  AbsoluteRotation absRotaion(robot, std::move(absCondition), rotationPidGain, calHead);
  absRotaion.run();

  // auto relCondition = std::maek_unique<RelativeAngleCondition>(robot, calHead, 0.0);
  // RelativeRotation relCondition(robot, std::move(relCondition), rotationPidGain, calHead);
  // relRotation.run();
}
