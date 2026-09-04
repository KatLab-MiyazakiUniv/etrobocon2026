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
   * @param _robot ロボット
   * @param _map ETラリーマップ
   * @param _mapData ゲート情報
   * @param _targetSpeed 通常Straightの速度
   * @param _squareTrackingSpeed 正方形追従時の速度
   * @param _squareTargetX 正方形の目標X座標
   * @param _rotationPid 回頭PID
   * @param _rightPid 右車輪速度PID
   * @param _leftPid 左車輪速度PID
   * @param _straightAnglePid Straight角度補正PID
   * @param _squareTrackingPid 正方形追従PID
   */
  RouteFollower(
      Robot& _robot,
      const EtRallyMap& _map,
      const MapData& _mapData,
      double _targetSpeed,
      double _squareTrackingSpeed,
      int _squareTargetX,
      const Pid::PidGain& _rotationPid,
      const Pid::PidGain& _rightPid,
      const Pid::PidGain& _leftPid,
      const Pid::PidGain& _straightAnglePid,
      const Pid::PidGain& _squareTrackingPid);

  /**
   * @brief 経路探索結果に従って走行する
   *
   * 通常区間:
   *   Straight
   *
   * ゲート通過区間:
   *   ゲート125mm手前までStraight
   *   ↓
   *   SquareTracking 250mm
   *   ↓
   *   残りStraight
   *
   * @param route 経路探索結果
   */
  void run(
      const std::vector<RouteState>& route);

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
   * @brief 通常Straight速度
   */
  double targetSpeed;

  /**
   * @brief 正方形追従速度
   */
  double squareTrackingSpeed;

  /**
   * @brief 正方形を合わせる画像上のX座標
   */
  int squareTargetX;

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
   * @brief 正方形追従PID
   */
  Pid::PidGain squareTrackingPid;

  /**
   * @brief Directionを方位角へ変換する
   * @param direction 方向
   * @return 方位角[deg]
   */
  double directionToHeading(
      Direction direction) const;

  /**
   * @brief 回頭角度を計算する
   * @param from 現在方向
   * @param to 目標方向
   * @return 回頭角度[deg]
   */
  double calculateRotationAngle(
      Direction from,
      Direction to) const;

  /**
   * @brief 2つの経路状態間の距離を計算する
   * @param from 開始状態
   * @param to 終了状態
   * @return 距離[mm]
   */
  double calculateDistance(
      const RouteState& from,
      const RouteState& to) const;

  /**
   * @brief 回頭する
   * @param angle 回頭角度[deg]
   */
  void rotate(
      double angle);

  /**
   * @brief 通常Straight
   * @param distance 走行距離[mm]
   */
  void straight(
      double distance);

  /**
   * @brief 正方形を使用したCameraTracking
   * @param distance 走行距離[mm]
   */
  void straightWithSquareCorrection(
      double distance);

  /**
   * @brief 指定した直進区間内に存在するゲートを探す
   *
   * 経路圧縮後でも判定できるように、
   * GatePassのentranceとexitがfrom-to区間内に
   * 存在するか確認する。
   *
   * @param from 区間開始
   * @param to 区間終了
   * @return ゲート。存在しない場合nullptr
   */
  const Gate* findGate(
      const RouteState& from,
      const RouteState& to) const;

  /**
   * @brief 開始地点からゲート中央までの距離を計算する
   * @param from 区間開始
   * @param gate ゲート
   * @return 距離[mm]
   */
  double calculateDistanceToGate(
      const RouteState& from,
      const Gate& gate) const;

  /**
   * @brief ゲートを含む直進区間を走行する
   *
   * ゲート手前まで通常Straightし、
   * ゲート付近のみ正方形CameraTrackingする。
   *
   * @param from 区間開始
   * @param to 区間終了
   * @param distance 区間全体の距離[mm]
   */
  void runGateSegment(
      const RouteState& from,
      const RouteState& to,
      double distance);
};

#endif