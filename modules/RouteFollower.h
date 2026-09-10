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

class RouteFollower {
 public:
  /**
   * @brief コンストラクタ
   */
  RouteFollower(
      Robot& _robot,
      const EtRallyMap& _map,
      const MapData& _mapData,
      double _targetSpeed,
      const Pid::PidGain& _rotationPid,
      const Pid::PidGain& _rightPid,
      const Pid::PidGain& _leftPid,
      const Pid::PidGain& _straightAnglePid,
      const Pid::PidGain& _squareAnglePid);

  /**
   * @brief 経路に従って走行する
   */
  void run(
      const std::vector<RouteState>& route);

 private:
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
   * @brief 2地点間の距離を計算する
   */
  double calculateDistance(
      const RouteState& from,
      const RouteState& to) const;

  /**
   * @brief 回頭する
   */
  void rotate(
      double angle);

  /**
   * @brief Straightで走行する
   *
   * Straight終了後は、
   * 次回のSquare検出で追跡履歴をリセットする。
   */
  void straight(
      double distance);

  /**
   * @brief 正方形を利用して向きを補正する
   */
  bool adjustAngleWithSquare();

  /**
   * @brief 現在の直進区間内に存在するゲートを探す
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
   * @brief 現在地点からゲート中心までの距離を計算する
   */
  double calculateDistanceToGate(
      const RouteState& from,
      const Gate& gate) const;

  /**
   * @brief ゲート区間を走行する
   */
  void runGateSegment(
      const RouteState& from,
      const RouteState& to,
      double distance);

 private:
  /**
   * @brief ロボット
   */
  Robot& robot;

  /**
   * @brief ETラリーマップ
   */
  const EtRallyMap& map;

  /**
   * @brief ゲート情報
   */
  const MapData& mapData;

  /**
   * @brief Straight速度[mm/s]
   */
  double targetSpeed;

  /**
   * @brief 回頭PID
   */
  Pid::PidGain rotationPid;

  /**
   * @brief 右車輪PID
   */
  Pid::PidGain rightPid;

  /**
   * @brief 左車輪PID
   */
  Pid::PidGain leftPid;

  /**
   * @brief Straight角度PID
   */
  Pid::PidGain straightAnglePid;

  /**
   * @brief Square角度補正PID
   */
  Pid::PidGain squareAnglePid;

  /**
   * @brief 次回Square検出時に追跡履歴をリセットするか
   *
   * Straightを実行するとtrueになる。
   * SquareAngleAdjustment開始時に消費されfalseになる。
   */
  bool squareTrackingResetRequired;
};

#endif