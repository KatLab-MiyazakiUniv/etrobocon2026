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
   */
  RouteFollower(Robot& _robot, const EtRallyMap& _map, const MapData& _mapData, double _targetSpeed,
                const Pid::PidGain& _rotationPid, const Pid::PidGain& _rightPid,
                const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid);

  /**
   * @brief 経路に従って走行する
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
   * @brief Directionを方位角へ変換
   */
  double directionToHeading(Direction direction) const;

  /**
   * @brief 必要回頭角を計算
   */
  double calculateRotationAngle(Direction from, Direction to) const;

  /**
   * @brief 2地点間の実距離を計算
   */
  double calculateDistance(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 相対回頭
   */
  void rotate(double angle);

  /**
   * @brief 直進
   */
  void straight(double distance);

  /**
   * @brief 正方形を検出して位置情報を取得
   *
   * @param result 検出結果
   * @return 検出成功時true
   */
  bool detectSquare(SquareAngleAdjustment::Result& result);

  /**
   * @brief 内側ゲート攻略
   */
  void runGateSegment(const RouteState& from, const RouteState& to, double distance);

  /**
   * @brief 区間に存在するゲートを検索
   */
  const Gate* findGate(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 外周ゲート判定
   */
  bool isOuterGate(const Gate& gate) const;

  /**
   * @brief 区間開始地点からゲート中心までの距離
   */
  double calculateDistanceToGate(const RouteState& from, const Gate& gate) const;
};

#endif