/**
 * @file   RouteFollower.h
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#ifndef ROUTE_FOLLOWER_H
#define ROUTE_FOLLOWER_H

#include <vector>

#include "EtRallyMap.h"
#include "MapData.h"
#include "Pid.h"
#include "Robot.h"
#include "RouteTypes.h"
#include "SquareAngleAdjustment.h"

class RouteFollower {
 public:
  /**
   * @brief コンストラクタ
   *
   * @param _robot ロボット
   * @param _map ETラリーの実座標マップ
   * @param _mapData ゲート情報
   * @param _targetSpeed 直進速度[mm/s]
   * @param _rotationPid 回頭PID
   * @param _rightPid 右車輪速度PID
   * @param _leftPid 左車輪速度PID
   * @param _straightAnglePid 直進角度PID
   */
  RouteFollower(Robot& _robot, const EtRallyMap& _map, const MapData& _mapData, double _targetSpeed,
                const Pid::PidGain& _rotationPid, const Pid::PidGain& _rightPid,
                const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid);

  /**
   * @brief 経路に従って走行する
   *
   * @param route 経路探索結果
   */
  void run(const std::vector<RouteState>& route);

 private:
  Robot& robot;

  const EtRallyMap& map;

  const MapData& mapData;

  double targetSpeed;

  Pid::PidGain rotationPid;

  Pid::PidGain rightPid;

  Pid::PidGain leftPid;

  Pid::PidGain straightAnglePid;

  /**
   * @brief Directionを方位角へ変換する
   */
  double directionToHeading(Direction direction) const;

  /**
   * @brief 必要な相対回頭角度を計算する
   */
  double calculateRotationAngle(Direction from, Direction to) const;

  /**
   * @brief 2地点間の走行距離を計算する
   */
  double calculateDistance(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 相対回頭する
   */
  void rotate(double angle);

  /**
   * @brief 指定距離を直進する
   */
  void straight(double distance);

  /**
   * @brief 正方形を1回検出し、
   *        角度・距離を取得する
   *
   * @param result 検出結果
   * @return 成功時true
   */
  bool detectSquare(SquareAngleAdjustment::Result& result);

  /**
   * @brief ゲート区間を走行する
   */
  void runGateSegment(const RouteState& from, const RouteState& to, double distance);

  /**
   * @brief 現在区間のゲートを取得する
   */
  const Gate* findGate(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 外周ゲートか判定する
   */
  bool isOuterGate(const Gate& gate) const;

  /**
   * @brief 区間開始位置からゲート中央までの距離
   */
  double calculateDistanceToGate(const RouteState& from, const Gate& gate) const;
};

#endif