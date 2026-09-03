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

class RouteFollower {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot ロボットインスタンス
   * @param _map ETラリーマップ
   * @param _targetSpeed 通常走行速度
   * @param _qrTrackingSpeed QR追従走行速度
   * @param _qrTargetX QRコードの目標X座標
   * @param _rotationPid 回頭用PIDゲイン
   * @param _rightPid 右車輪用PIDゲイン
   * @param _leftPid 左車輪用PIDゲイン
   * @param _straightAnglePid 直進角度補正用PIDゲイン
   * @param _qrTrackingPid QR追従用PIDゲイン
   */
  RouteFollower(
      Robot& _robot,
      const EtRallyMap& _map,
      double _targetSpeed,
      double _qrTrackingSpeed,
      int _qrTargetX,
      const Pid::PidGain& _rotationPid,
      const Pid::PidGain& _rightPid,
      const Pid::PidGain& _leftPid,
      const Pid::PidGain& _straightAnglePid,
      const Pid::PidGain& _qrTrackingPid);

  /**
   * @brief QR補正を使用せず経路を走行する
   * @param route 経路探索結果
   */
  void run(
      const std::vector<RouteState>& route);

  /**
   * @brief 指定したゲートへの経路を走行する
   *
   * 最後の走行区間でゲート通過処理を行う。
   * 内側ゲートの場合はQR補正を行う。
   *
   * @param route 経路探索結果
   * @param targetGate 通過対象ゲート
   */
  void run(
      const std::vector<RouteState>& route,
      const Gate& targetGate);

 private:
  /**
   * @brief Directionを角度へ変換する
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
   * @brief 2地点間の走行距離を計算する
   * @param from 開始地点
   * @param to 終了地点
   * @return 走行距離[mm]
   */
  double calculateDistance(
      const RouteState& from,
      const RouteState& to) const;

  /**
   * @brief 指定角度だけ回頭する
   * @param angle 回頭角度[deg]
   */
  void rotate(
      double angle);

  /**
   * @brief 指定距離を通常直進する
   * @param distance 走行距離[mm]
   */
  void straight(
      double distance);

  /**
   * @brief QRコードを使用して指定距離を追従走行する
   * @param distance 走行距離[mm]
   */
  void straightWithQrCorrection(
      double distance);

  /**
   * @brief QRコードを1回検出する
   * @return true QRコードを検出した
   * @return false QRコードを検出できなかった
   */
  bool detectQrCode();

  /**
   * @brief ゲートが外周ゲートか判定する
   * @param gate 判定対象ゲート
   * @return true 外周ゲート
   * @return false 内側ゲート
   */
  bool isOuterGate(
      const Gate& gate) const;

  /**
   * @brief 区間開始地点からゲートまでの距離を計算する
   * @param from 区間開始地点
   * @param gate 通過対象ゲート
   * @return ゲートまでの距離[mm]
   */
  double calculateDistanceToGate(
      const RouteState& from,
      const Gate& gate) const;

  /**
   * @brief ゲートを含む最後の区間を走行する
   *
   * 内側ゲートの場合、
   * ゲート125mm手前まで走行し、一時停止後にQRを検出する。
   * 検出成功時はQR追従、失敗時は通常直進する。
   *
   * @param from 区間開始地点
   * @param to 区間終了地点
   * @param distance 区間全体の距離[mm]
   * @param gate 通過対象ゲート
   */
  void runGateSegment(
      const RouteState& from,
      const RouteState& to,
      double distance,
      const Gate& gate);

  /** ロボット */
  Robot& robot;

  /** ETラリーマップ */
  const EtRallyMap& map;

  /** 通常走行速度 */
  double targetSpeed;

  /** QR追従走行速度 */
  double qrTrackingSpeed;

  /** QRコードの目標X座標 */
  int qrTargetX;

  /** 回頭用PIDゲイン */
  Pid::PidGain rotationPid;

  /** 右車輪用PIDゲイン */
  Pid::PidGain rightPid;

  /** 左車輪用PIDゲイン */
  Pid::PidGain leftPid;

  /** Straight角度補正用PIDゲイン */
  Pid::PidGain straightAnglePid;

  /** QR追従用PIDゲイン */
  Pid::PidGain qrTrackingPid;
};

#endif  // ROUTE_FOLLOWER_H