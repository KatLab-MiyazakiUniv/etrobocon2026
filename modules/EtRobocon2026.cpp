/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "RealNetworkSystem.h"
#include "SocketClient.h"
#include "Robot.h"
#include "AreaMaster.h"

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  RealNetworkSystem netSys;
  SocketClient cameraSocketClient(netSys);
  Robot robot(cameraSocketClient);
  robot.setCourse(RUNNING_COURSE);

  if(!cameraSocketClient.connectToServer()) {
    Logger::error("EtRobocon2026:カメラサーバーへの接続に失敗しました");
    return;
  }

  AreaMaster areaMaster(robot, Area::LineTrace);
  areaMaster.run();

  cameraSocketClient.disconnectFromServer();
}
