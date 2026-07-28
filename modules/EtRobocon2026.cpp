/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

#include <string>
#include <vector>

#include "BaseMotion.h"
#include "Logger.h"
#include "MotionParser.h"
#include "RealNetworkSystem.h"
#include "Robot.h"
#include "SocketClient.h"

void EtRobocon2026::start()
{
  Logger::info("LineTraceRight start");

  RealNetworkSystem networkSystem;
  SocketClient cameraSocketClient(networkSystem);
  Robot robot(cameraSocketClient);

  std::string commandFilePath = "etrobocon2026/datafiles/commands/Areas/LineTraceRight.csv";

  std::vector<BaseMotion*> motionList = MotionParser::createMotionList(robot, commandFilePath);

  if(motionList.empty()) {
    Logger::error("LineTraceRight.csvからモーションを生成できませんでした");
    return;
  }

  for(BaseMotion* motion : motionList) {
    if(motion == nullptr) {
      continue;
    }

    motion->run();
  }

  for(BaseMotion* motion : motionList) {
    delete motion;
  }

  Logger::info("LineTraceRight finish");
}