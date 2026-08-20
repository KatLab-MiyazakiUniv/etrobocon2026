/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

#include <vector>

#include "EtRallyMap.h"
#include "RouteFollower.h"

void EtRobocon2026::start()
{
  Logger::info("RouteFollower test start");

  EtRallyMap map;

  constexpr double TARGET_SPEED = 300.0;

  Pid::PidGain rotationPid = { 0.8, 0.0, 0.0 };

  Pid::PidGain straightAnglePid = { 0.8, 0.0, 0.0 };

  /*
   * テスト経路
   *
   * (1,1) → (3,1)
   *              ↓
   *            (3,3)
   *              ↓
   *            (5,3)
   */
  std::vector<RouteFollower::GridPoint> route = { { 1, 1 }, { 3, 1 }, { 3, 3 }, { 5, 3 } };

  RouteFollower follower(robot, map, TARGET_SPEED, rotationPid, straightAnglePid);

  follower.run(route, RouteFollower::Direction::RIGHT);

  Logger::info("RouteFollower test finished");
}