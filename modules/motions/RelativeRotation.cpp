/**
 * @file   RelativeRotation.cpp
 * @brief  相対角度で回頭動作を実行するクラス
 * @author okuyama0528
 */
#include "EtRobocon2026.h"
#include "RelativeRotation.h"
#include <iostream>

void EtRobocon2026::start()
{
  std::cout << "Hello KATLAB" << std::endl;

  RelativeRotation rotation(robot, 80.0);

  rotation.prepare();

  while(!rotation.isFinished()) {
    rotation.executeStep();
  }

  rotation.finish();
}