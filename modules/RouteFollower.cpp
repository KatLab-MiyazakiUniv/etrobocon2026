/**
 * @file   RouteFollower.cpp
 * @brief  経路探索結果に従って回頭と直進を実行するクラス
 * @author yutaro-1214
 */

#include "RouteFollower.h"

#include <cmath>
#include <memory>

#include "DistanceCondition.h"
#include "Logger.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "Straight.h"

RouteFollower::RouteFollower(Robot& _robot, EtRallyMap& _map, double _targetSpeed,
                             const Pid::PidGain& _rotationPid,
                             const Pid::PidGain& _straightAnglePid)
  : robot(_robot),
    map(_map),
    targetSpeed(_targetSpeed),
    rotationPid(_rotationPid),
    straightAnglePid(_straightAnglePid)
{
  LOG_CREATE("RouteFollower");
}

void RouteFollower::run(const std::vector<GridPoint>& route, Direction startDirection)
{
  if(route.size() < 2) {
    Logger::info("RouteFollower: route is too short");
    return;
  }

  Direction currentDirection = startDirection;

  for(std::size_t i = 1; i < route.size(); ++i) {
    const GridPoint& from = route[i - 1];
    const GridPoint& to = route[i];

    Direction nextDirection = calculateDirection(from, to);

    double rotationAngle = calculateRotationAngle(currentDirection, nextDirection);

    double distance = calculateDistance(from, to);

    Logger::printfLog(Logger::INFO, "RouteFollower: (%d,%d) -> (%d,%d), angle=%.2f, distance=%.2f",
                      from.x, from.y, to.x, to.y, rotationAngle, distance);

    /*
     * 必要な場合だけ回頭
     */
    if(std::abs(rotationAngle) > 0.001) {
      rotate(rotationAngle);
    }

    /*
     * その区間を直進
     */
    if(distance > 0.0) {
      straight(distance);
    }

    currentDirection = nextDirection;
  }
}

RouteFollower::Direction RouteFollower::calculateDirection(const GridPoint& from,
                                                           const GridPoint& to) const
{
  /*
   * EtRallyMapでは
   *
   * gridX増加 → X方向
   * gridY増加 → 下方向
   */

  if(to.x > from.x) {
    return Direction::RIGHT;
  }

  if(to.x < from.x) {
    return Direction::LEFT;
  }

  if(to.y > from.y) {
    return Direction::DOWN;
  }

  return Direction::UP;
}

double RouteFollower::calculateRotationAngle(Direction currentDirection,
                                             Direction targetDirection) const
{
  int current = static_cast<int>(currentDirection);
  int target = static_cast<int>(targetDirection);

  int diff = target - current;

  /*
   * 180度を超える場合は逆方向に回した方が短い
   */
  if(diff > 2) {
    diff -= 4;
  } else if(diff < -2) {
    diff += 4;
  }

  return diff * 90.0;
}

double RouteFollower::calculateDistance(const GridPoint& from, const GridPoint& to) const
{
  EtRallyMap::Node fromNode = map.getNode(from.x, from.y);

  EtRallyMap::Node toNode = map.getNode(to.x, to.y);

  /*
   * X方向への移動
   */
  if(from.y == to.y) {
    return std::abs(toNode.x - fromNode.x);
  }

  /*
   * Y方向への移動
   */
  if(from.x == to.x) {
    return std::abs(toNode.y - fromNode.y);
  }

  /*
   * 経路圧縮後の経路は縦か横だけを想定しているため、
   * 斜め移動は不正
   */
  Logger::printfLog(Logger::ERROR, "RouteFollower: diagonal route (%d,%d) -> (%d,%d)", from.x,
                    from.y, to.x, to.y);

  return 0.0;
}

void RouteFollower::rotate(double angle)
{
  constexpr double ROTATION_TOLERANCE = 2.0;

  auto condition = std::make_unique<RelativeAngleCondition>(robot, angle, ROTATION_TOLERANCE);

  RelativeRotation rotation(robot, std::move(condition), rotationPid, angle);

  rotation.run();
}

void RouteFollower::straight(double distance)
{
  auto condition = std::make_unique<DistanceCondition>(robot, distance);

  Straight straightMotion(robot, std::move(condition), targetSpeed, straightAnglePid, true);

  straightMotion.run();
}