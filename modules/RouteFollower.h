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
   * @param _map ETラリーマップ
   * @param _mapData ゲート情報
   * @param _targetSpeed 直進速度[mm/s]
   * @param _rotationPid 通常回頭用PID
   * @param _squareRotationPid 正方形補正回頭用PID
   * @param _rightPid 右モータ速度PID
   * @param _leftPid 左モータ速度PID
   * @param _straightAnglePid 直進角度PID
   */
  RouteFollower(
      Robot& _robot,
      const EtRallyMap& _map,
      const MapData& _mapData,
      double _targetSpeed,
      const Pid::PidGain& _rotationPid,
      const Pid::PidGain& _squareRotationPid,
      const Pid::PidGain& _rightPid,
      const Pid::PidGain& _leftPid,
      const Pid::PidGain& _straightAnglePid);

  /**
   * @brief 経路に従って走行する
   *
   * @param route 経路探索結果
   */
  void run(
      const std::vector<RouteState>& route);

 private:
  Robot& robot;

  const EtRallyMap& map;

  const MapData& mapData;

  double targetSpeed;

  /**
   * @brief 通常の90度回頭などで使用するPID
   */
  Pid::PidGain rotationPid;

  /**
   * @brief 正方形を利用した補正回頭用PID
   */
  Pid::PidGain squareRotationPid;

  Pid::PidGain rightPid;

  Pid::PidGain leftPid;

  Pid::PidGain straightAnglePid;

  /**
   * @brief Directionを方位角へ変換する
   */
  double directionToHeading(
      Direction direction) const;

  /**
   * @brief 必要な回頭角度を計算する
   */
  double calculateRotationAngle(
      Direction from,
      Direction to) const;

  /**
   * @brief 2地点間の実距離を計算する
   */
  double calculateDistance(
      const RouteState& from,
      const RouteState& to) const;

  /**
   * @brief 通常回頭を行う
   *
   * rotationPidを使用する。
   */
  void rotate(
      double angle);

  /**
   * @brief 正方形補正用回頭を行う
   *
   * squareRotationPidを使用する。
   */
  void rotateForSquare(
      double angle);

  /**
   * @brief 指定距離を直進する
   */
  void straight(
      double distance);

  /**
   * @brief 正方形を検出する
   *
   * @param result 検出・計算結果
   * @return 成功時true
   */
  bool detectSquare(
      SquareAngleAdjustment::Result& result);

  /**
   * @brief ゲートを含む区間を走行する
   */
  void runGateSegment(
      const RouteState& from,
      const RouteState& to,
      double distance);

  /**
   * @brief 区間内に存在するゲートを探す
   */
  const Gate* findGate(
      const RouteState& from,
      const RouteState& to) const;

  /**
   * @brief 外周ゲートか判定する
   */
  bool isOuterGate(
      const Gate& gate) const;

  /**
   * @brief 区間開始地点からゲート中央までの距離を計算する
   */
  double calculateDistanceToGate(
      const RouteState& from,
      const Gate& gate) const;
};

#endif