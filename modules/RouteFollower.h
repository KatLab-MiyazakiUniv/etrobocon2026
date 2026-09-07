/**
 * @file   RouteFollower.h
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#ifndef ROUTE_FOLLOWER_H
#define ROUTE_FOLLOWER_H

#include <vector>

#include "EtRallyMap.h"
#include "Pid.h"
#include "Robot.h"
#include "RouteTypes.h"

/**
 * @class RouteFollower
 * @brief 経路探索結果に従って回頭と直進を行うクラス
 *
 * 90度回頭を行うたびに、
 * 正方形を利用して角度誤差を補正する。
 */
class RouteFollower {
 public:
  /**
   * @brief コンストラクタ
   *
   * @param _robot ロボット
   * @param _map ETラリーマップ
   * @param _targetSpeed Straight走行速度
   * @param _rotationPid 回頭PID
   * @param _rightPid 右車輪速度PID
   * @param _leftPid 左車輪速度PID
   * @param _straightAnglePid Straight角度補正PID
   * @param _squareAnglePid 正方形角度補正PID
   */
  RouteFollower(Robot& _robot, const EtRallyMap& _map, double _targetSpeed,
                const Pid::PidGain& _rotationPid, const Pid::PidGain& _rightPid,
                const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid,
                const Pid::PidGain& _squareAnglePid);

  /**
   * @brief 経路を走行する
   *
   * @param route 経路探索結果
   */
  void run(const std::vector<RouteState>& route);

 private:
  /**
   * @brief Directionを角度へ変換する
   */
  double directionToHeading(Direction direction) const;

  /**
   * @brief 回頭角度を計算する
   */
  double calculateRotationAngle(Direction from, Direction to) const;

  /**
   * @brief 2地点間の走行距離を計算する
   */
  double calculateDistance(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 指定角度だけ回頭する
   */
  void rotate(double angle);

  /**
   * @brief 90度単位で回頭し、
   *        90度回頭ごとに正方形角度補正を行う
   */
  void rotateWithSquareCorrection(double angle);

  /**
   * @brief Straight走行する
   */
  void straight(double distance);

  /**
   * @brief 正方形による角度補正
   *
   * @return true 補正成功
   * @return false 正方形未検出等で補正スキップ
   */
  bool adjustAngleWithSquare();

  /**
   * @brief ロボット
   */
  Robot& robot;

  /**
   * @brief ETラリーマップ
   */
  const EtRallyMap& map;

  /**
   * @brief Straight走行速度
   */
  double targetSpeed;

  /**
   * @brief 回頭PID
   */
  Pid::PidGain rotationPid;

  /**
   * @brief 右車輪速度PID
   */
  Pid::PidGain rightPid;

  /**
   * @brief 左車輪速度PID
   */
  Pid::PidGain leftPid;

  /**
   * @brief Straight角度補正PID
   */
  Pid::PidGain straightAnglePid;

  /**
   * @brief 正方形角度補正PID
   */
  Pid::PidGain squareAnglePid;
};

#endif