/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "Robot.h"
#include "SocketClient.h"
#include "RealNetworkSystem.h"
#include "Straight.h"
#include "DistanceCondition.h"
#include "Pid.h"


void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");


  RealNetworkSystem real;

  SocketClient client(real);

  Robot robot(client);


  // PID設定
  Pid::PidGain rightPid(
      0.016,
      0.005,
      0.0015
  );

  Pid::PidGain leftPid(
      0.016,
      0.0045,
      0.0015
  );

  Pid::PidGain anglePid(
      0.036,
      0.012,
      0.03
  );


  // 指定距離だけ直進
  Straight straight(
      robot,
      std::make_unique<DistanceCondition>(
          robot,
          1000.0   // mm
      ),
      300,       // 速度
      rightPid,
      leftPid,
      anglePid,
      true
  );


  straight.run();


  Logger::info("Straight finished");
}