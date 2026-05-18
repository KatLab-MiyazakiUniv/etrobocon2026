/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author yutaro-1214
 */

#include "EtRobocon2026.h"

#include <iostream>
#include <memory>

#include "Rotation.h"
#include "motions/continuation_conditions/AbsoluteAngleCondition.h"

void EtRobocon2026::start()
{
  Robot robot;

  std::cout << "Rotation Test Start" << std::endl;

  // 方位角リセット
  robot.getIMUControllerInstance().resetAzimuth();

  // 30度回転条件
  auto condition = std::make_unique<AbsoluteAngleCondition>(robot, 30.0f);

  // 回転動作生成
  Rotation rotation(robot, std::move(condition), 30.0f);

  // 実行
  rotation.run();

  std::cout << "Rotation Test End" << std::endl;
}