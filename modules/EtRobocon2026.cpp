/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */
#include "EtRobocon2026.h"
#include "RelativeRotation.h"
#include "AbsoluteRotation.h"
#include "RelativeAngleContinuationCondition.h"
#include "AbsoluteAngleContinuationCondition.h"
#include <iostream>
#include <memory>

void EtRobocon2026::start()
{
  std::cout << "Hello KATLAB" << std::endl;

  Robot robot;

  Pid::PidGain anglePidGain(0.8, 0.0, 0.05);

  // 相対角度回頭（現在角度から80度回転）
  auto relCondition = std::make_unique<RelativeAngleContinuationCondition>(robot, 80.0);
  RelativeRotation relRotation(robot, std::move(relCondition), anglePidGain, 80.0);
  relRotation.run();

  // 絶対角度回頭（北を0度として90度の方向を向く）
  // auto absCondition = std::make_unique<AbsoluteAngleContinuationCondition>(robot, 90.0);
  // AbsoluteRotation absRotation(robot, std::move(absCondition), anglePidGain, 90.0);
  // absRotation.run();
}
