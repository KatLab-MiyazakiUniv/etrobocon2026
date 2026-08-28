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
   * @param robot ロボット
   * @param map ETラリーのマップ
   * @param targetSpeed 直進速度
   * @param rotationPid 回頭PID
   * @param rightPid 右車輪PID
   * @param leftPid 左車輪PID
   * @param straightAnglePid 直進角度補正PID
   */
  RouteFollower(Robot& robot, const EtRallyMap& map, double targetSpeed,
                const Pid::PidGain& rotationPid, const Pid::PidGain& rightPid,
                const Pid::PidGain& leftPid, const Pid::PidGain& straightAnglePid);

  /**
   * @brief 圧縮済み経路を通常走行する
   * @param route 経路探索結果
   */
  void run(const std::vector<RouteState>& route);

  /**
   * @brief 圧縮済み経路を走行する
   *
   * useQrCorrectionがtrueの場合、
   * 最後の直進区間をゲート通過区間として扱い、
   * その直前にQRコードを検知する。
   *
   * QRコードを検知できた場合はCameraTracking、
   * 検知できなかった場合は通常のStraightで走行する。
   *
   * @param route 経路探索結果
   * @param useQrCorrection QR補正を使用するかどうか
   */
  void run(const std::vector<RouteState>& route, bool useQrCorrection);

 private:
  /**
   * @brief Directionを方位角へ変換する
   */
  double directionToHeading(Direction direction) const;

  /**
   * @brief 現在方向から目標方向への相対回頭角を求める
   */
  double calculateRotationAngle(Direction from, Direction to) const;

  /**
   * @brief 2地点間の実際の距離をEtRallyMapから求める
   */
  double calculateDistance(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 指定角度だけ回頭する
   */
  void rotate(double angle);

  /**
   * @brief 指定距離だけ通常走行する
   */
  void straight(double distance);

  /**
   * @brief 指定距離をQRコードによるカメラ補正付きで走行する
   *
   * QRコードを事前に検知できた場合に使用する。
   */
  void straightWithQrCorrection(double distance);

  /**
   * @brief QRコードを検知する
   * @return true QRコードを検知できた場合
   * @return false QRコードを検知できなかった場合
   */
  bool detectQrCode();

  Robot& robot;
  const EtRallyMap& map;

  double targetSpeed;

  Pid::PidGain rotationPid;
  Pid::PidGain rightPid;
  Pid::PidGain leftPid;
  Pid::PidGain straightAnglePid;
};

#endif  // ROUTE_FOLLOWER_H
