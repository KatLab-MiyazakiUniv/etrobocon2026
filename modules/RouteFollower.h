/**
 * @file   RouteFollower.h
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#ifndef ROUTE_FOLLOWER_H
#define ROUTE_FOLLOWER_H

#include <vector>
#include <cmath>
#include <memory>
#include "AngleNormalizer.h"
#include "ClockUtil.h"
#include "DistanceCondition.h"
#include "EtRallyMap.h"
#include "Logger.h"
#include "MapData.h"
#include "Pid.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "Robot.h"
#include "RouteTypes.h"
#include "SocketProtocol.h"
#include "SquareAngleAdjustment.h"
#include "Straight.h"
#include "SystemInfo.h"

class RouteFollower {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot ロボット
   * @param _map ETラリーマップ
   * @param _mapData ゲート情報
   * @param _targetSpeed 直進速度
   * @param _rotationPid 通常回頭PID
   * @param _squareRotationPid 正方形補正回頭PID
   * @param _rightPid 右モーターPID
   * @param _leftPid 左モーターPID
   * @param _straightAnglePid 直進角度PID
   */
  RouteFollower(Robot& _robot, const EtRallyMap& _map, const MapData& _mapData, double _targetSpeed,
                const Pid::PidGain& _rotationPid, const Pid::PidGain& _squareRotationPid,
                const Pid::PidGain& _rightPid, const Pid::PidGain& _leftPid,
                const Pid::PidGain& _straightAnglePid);

  /**
   * @brief 経路を走行する
   */
  void run(const std::vector<RouteState>& route);

 private:
  Robot& robot;

  const EtRallyMap& map;

  const MapData& mapData;

  double targetSpeed;

  Pid::PidGain rotationPid;

  Pid::PidGain squareRotationPid;

  Pid::PidGain rightPid;

  Pid::PidGain leftPid;

  Pid::PidGain straightAnglePid;

  /**
   * @brief Directionを角度へ変換する
   */
  double directionToHeading(Direction direction) const;

  /**
   * @brief 必要な回頭角度を計算する
   */
  double calculateRotationAngle(Direction from, Direction to) const;

  /**
   * @brief 2地点間の距離を計算する
   */
  double calculateDistance(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 通常回頭
   */
  void rotate(double angle);

  /**
   * @brief 正方形補正用回頭
   */
  void rotateForSquare(double angle);

  /**
   * @brief 直進
   */
  void straight(double distance);

  /**
   * @brief 正方形を検出し補正情報を取得する
   */
  bool detectSquare(SquareAngleAdjustment::Result& result);

  /**
   * @brief ゲートを含む区間を走行する
   *
   * @param from 区間開始
   * @param to 区間終了
   * @param distance 区間距離
   * @param rotatedAtSegmentStart
   *        この区間開始時に回頭したか
   */
  void runGateSegment(const RouteState& from, const RouteState& to, double distance,
                      bool rotatedAtSegmentStart);

  /**
   * @brief 区間に存在するゲートを取得する
   */
  const Gate* findGate(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 外周ゲートか
   */
  bool isOuterGate(const Gate& gate) const;

  /**
   * @brief 区間開始位置からゲート中央までの距離
   */
  double calculateDistanceToGate(const RouteState& from, const Gate& gate) const;
};

#endif  // ROUTE_FOLLOWER_H