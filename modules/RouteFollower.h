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
   * @param robot ロボット
   * @param map ETラリーの実座標マップ
   * @param mapData ゲート情報
   * @param targetSpeed 通常直進速度
   * @param qrTrackingSpeed QR追従走行速度
   * @param qrTargetX QR追従時の目標X座標
   * @param rotationPid 回頭PID
   * @param rightPid 右車輪PID
   * @param leftPid 左車輪PID
   * @param straightAnglePid 通常直進角度補正PID
   * @param qrTrackingPid QR追従PID
   */
  RouteFollower(
      Robot& robot,
      const EtRallyMap& map,
      const MapData& mapData,
      double targetSpeed,
      double qrTrackingSpeed,
      int qrTargetX,
      const Pid::PidGain& rotationPid,
      const Pid::PidGain& rightPid,
      const Pid::PidGain& leftPid,
      const Pid::PidGain& straightAnglePid,
      const Pid::PidGain& qrTrackingPid);

  /**
   * @brief QR補正を使用せず圧縮済み経路を走行する
   * @param route 経路探索結果
   */
  void run(const std::vector<RouteState>& route);

  /**
   * @brief 圧縮済み経路を走行する
   *
   * useQrCorrectionがtrueの場合、
   * 最後の区間をゲート通過区間として処理する。
   *
   * 内側ゲートの場合はゲート125mm手前でQRコードを確認し、
   * 検出できた場合は250mmをQR追従走行する。
   *
   * QRコードを検出できなかった場合は250mm通常直進する。
   *
   * 外周ゲートの場合はQR補正を使用せず、
   * 区間全体を通常直進する。
   *
   * @param route 経路探索結果
   * @param useQrCorrection QR補正を使用するか
   */
  void run(
      const std::vector<RouteState>& route,
      bool useQrCorrection);

 private:
  /**
   * @brief Directionを方位角へ変換する
   */
  double directionToHeading(
      Direction direction) const;

  /**
   * @brief 現在方向から目標方向への相対回頭角を求める
   */
  double calculateRotationAngle(
      Direction from,
      Direction to) const;

  /**
   * @brief 2地点間の実距離を求める
   */
  double calculateDistance(
      const RouteState& from,
      const RouteState& to) const;

  /**
   * @brief 指定角度だけ回頭する
   */
  void rotate(double angle);

  /**
   * @brief 指定距離だけ通常直進する
   */
  void straight(double distance);

  /**
   * @brief 指定距離だけQR追従走行する
   */
  void straightWithQrCorrection(double distance);

  /**
   * @brief QRコードを1回検出する
   *
   * CameraTrackingを開始するかどうかの判定に使用する。
   *
   * @return true QRコードを検出した
   * @return false QRコードを検出できなかった
   */
  bool detectQrCode();

  /**
   * @brief 指定区間を通過するゲートを取得する
   *
   * MapDataのGatePassと、
   * from -> to の区間を比較して対象ゲートを取得する。
   *
   * @return Gateへのポインタ
   * @return nullptr 対応するゲートがない場合
   */
  const Gate* findGate(
      const RouteState& from,
      const RouteState& to) const;

  /**
   * @brief ゲートが外周にあるか判定する
   */
  bool isOuterGate(const Gate& gate) const;

  /**
   * @brief 区間開始地点からゲート中心までの距離を求める
   */
  double calculateDistanceToGate(
      const RouteState& from,
      const Gate& gate) const;

  /**
   * @brief ゲート通過区間を走行する
   *
   * 外周ゲート:
   *   全区間Straight
   *
   * 内側ゲート:
   *   1. ゲート125mm手前までStraight
   *   2. QRコード検出
   *   3. 250mm CameraTrackingまたはStraight
   *   4. 残りをStraight
   */
  void runGateSegment(
      const RouteState& from,
      const RouteState& to,
      double distance);

  Robot& robot;

  const EtRallyMap& map;

  const MapData& mapData;

  double targetSpeed;

  double qrTrackingSpeed;

  int qrTargetX;

  Pid::PidGain rotationPid;

  Pid::PidGain rightPid;

  Pid::PidGain leftPid;

  Pid::PidGain straightAnglePid;

  Pid::PidGain qrTrackingPid;
};

#endif  // ROUTE_FOLLOWER_H