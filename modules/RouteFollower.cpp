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
#include "RelativeAngleContinuationCondition.h"
#include "RelativeRotation.h"
#include "Straight.h"

RouteFollower::RouteFollower(Robot& _robot, EtRallyMap& _map, double _targetSpeed,
                             const Pid::PidGain& _rotationPid, const Pid::PidGain& _rightPid,
                             const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid,
                             bool _useCompass)
  : robot(_robot),
    map(_map),
    targetSpeed(_targetSpeed),
    rotationPid(_rotationPid),
    rightPid(_rightPid),
    leftPid(_leftPid),
    straightAnglePid(_straightAnglePid),
    useCompass(_useCompass)
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

    if(std::abs(rotationAngle) > 0.001) {
      rotate(rotationAngle);
    }

    if(distance > 0.0) {
      straight(distance);
    }

    currentDirection = nextDirection;
  }
}

RouteFollower::Direction RouteFollower::calculateDirection(const GridPoint& from,
                                                           const GridPoint& to) const
{
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

  // X方向
  if(from.y == to.y) {
    return std::abs(toNode.x - fromNode.x);
  }

  // Y方向
  if(from.x == to.x) {
    return std::abs(toNode.y - fromNode.y);
  }

  Logger::printfLog(Logger::ERROR, "RouteFollower: diagonal route (%d,%d) -> (%d,%d)", from.x,
                    from.y, to.x, to.y);

  return 0.0;
}

void RouteFollower::rotate(double angle)
{
  auto condition = std::make_unique<RelativeAngleContinuationCondition>(robot, angle);

  RelativeRotation rotation(robot, std::move(condition), rotationPid, angle);

  rotation.run();
}

void RouteFollower::straight(double distance)
{
  auto condition = std::make_unique<DistanceCondition>(robot, distance);

  Straight straightMotion(robot, std::move(condition), targetSpeed, rightPid, leftPid,
                          straightAnglePid, useCompass);

  straightMotion.run();
}