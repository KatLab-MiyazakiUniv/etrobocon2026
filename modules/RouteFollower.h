/**
 * @file   RouteFollower.h
 * @brief  経路探索結果に従って回頭と直進を実行するクラス
 * @author yutaro-1214
 */

#ifndef ROUTE_FOLLOWER_H
#define ROUTE_FOLLOWER_H

#include <vector>

#include "EtRallyMap.h"
#include "Pid.h"
#include "Robot.h"

class RouteFollower {
 public:
  struct GridPoint {
    int x;
    int y;
  };

  enum class Direction { UP, RIGHT, DOWN, LEFT };

  RouteFollower(Robot& robot, EtRallyMap& map, double targetSpeed, const Pid::PidGain& rotationPid,
                const Pid::PidGain& rightPid, const Pid::PidGain& leftPid,
                const Pid::PidGain& straightAnglePid, bool useCompass);

  void run(const std::vector<GridPoint>& route, Direction startDirection);

 private:
  Direction calculateDirection(const GridPoint& from, const GridPoint& to) const;

  double calculateRotationAngle(Direction currentDirection, Direction targetDirection) const;

  double calculateDistance(const GridPoint& from, const GridPoint& to) const;

  void rotate(double angle);

  void straight(double distance);

  Robot& robot;
  EtRallyMap& map;

  double targetSpeed;

  Pid::PidGain rotationPid;

  Pid::PidGain rightPid;
  Pid::PidGain leftPid;
  Pid::PidGain straightAnglePid;

  bool useCompass;
};

#endif  // ROUTE_FOLLOWER_H