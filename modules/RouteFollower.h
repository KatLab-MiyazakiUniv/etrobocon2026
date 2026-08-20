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
  /**
   * @brief グリッド座標
   */
  struct GridPoint {
    int x;
    int y;
  };

  /**
   * @brief ロボットの進行方向
   */
  enum class Direction { UP, RIGHT, DOWN, LEFT };

  /**
   * @brief コンストラクタ
   *
   * @param robot ロボット
   * @param map ETラリーマップ
   * @param targetSpeed 直進速度
   * @param rotationPid 回頭PID
   * @param straightAnglePid 直進角度補正PID
   */
  RouteFollower(Robot& robot, EtRallyMap& map, double targetSpeed, const Pid::PidGain& rotationPid,
                const Pid::PidGain& straightAnglePid);

  /**
   * @brief 経路に従って走行する
   *
   * @param route 圧縮済み経路
   * @param startDirection 開始時の向き
   */
  void run(const std::vector<GridPoint>& route, Direction startDirection);

 private:
  /**
   * @brief 2地点間の進行方向を求める
   */
  Direction calculateDirection(const GridPoint& from, const GridPoint& to) const;

  /**
   * @brief 現在方向から次の方向への相対回頭角を求める
   */
  double calculateRotationAngle(Direction currentDirection, Direction targetDirection) const;

  /**
   * @brief マップ情報から2地点間の実距離を求める
   */
  double calculateDistance(const GridPoint& from, const GridPoint& to) const;

  /**
   * @brief 指定角度だけ回頭する
   */
  void rotate(double angle);

  /**
   * @brief 指定距離だけ直進する
   */
  void straight(double distance);

  Robot& robot;
  EtRallyMap& map;

  double targetSpeed;

  Pid::PidGain rotationPid;
  Pid::PidGain straightAnglePid;
};

#endif  // ROUTE_FOLLOWER_H