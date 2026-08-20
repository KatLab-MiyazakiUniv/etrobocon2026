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
  Logger::info("Route search + RouteFollower test start");

  constexpr double TARGET_SPEED = 300.0;

  Pid::PidGain rotationPid = { 0.8, 0.0, 0.0 };

  Pid::PidGain rightPid = { 0.007, 0.006, 0.003 };

  Pid::PidGain leftPid = { 0.007, 0.006, 0.003 };

  Pid::PidGain straightAnglePid = { 0.8, 0.0, 0.0 };

  constexpr bool USE_COMPASS = true;

  EtRallyMap etRallyMap;

  // ============================
  // ここで既存の経路探索を実行
  // ============================

  // 例:
  //
  // auto result = routePlanner.search(...);
  //
  // result.route
  // ↓
  // RouteFollower用に変換

  std::vector<RouteFollower::GridPoint> route;

  for(const auto& node : routeResult.route) {
    route.push_back({ node.x, node.y });
  }

  // ============================
  // 探索結果を実際に走行
  // ============================

  RouteFollower follower(robot, etRallyMap, TARGET_SPEED, rotationPid, rightPid, leftPid,
                         straightAnglePid, USE_COMPASS);

  follower.run(route, RouteFollower::Direction::RIGHT);

  Logger::info("Route search + RouteFollower test finished");
}