/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

void EtRobocon2026::start()
{
  std::cout << "Hello KATLAB" << std::endl;

  // IMU方位角リセット
  robot.getIMUControllerInstance().resetAzimuth();

  // 30度到達で終了する条件
  auto condition = std::make_unique<AbsoluteAngleCondition>(robot, 30.0f);

  // 回転動作生成
  Rotation rotation(robot, std::move(condition), 30.0f);

  // 動作開始準備
  rotation.prepare();

  // 条件を満たすまで回転
  while(rotation.shouldContinue()) {
    rotation.executeStep();
    std::cout << robot.getIMUControllerInstance().getAzimuth() << std::endl;
  }

  // 終了処理
  rotation.finish();

  std::cout << "Rotation Test End" << std::endl;
}