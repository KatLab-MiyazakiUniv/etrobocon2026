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
   * @param _robot ロボットインスタンス
   * @param _map ETラリーの実座標マップ
   * @param _mapData ゲート情報
   * @param _targetSpeed 通常走行時の目標速度[mm/s]
   * @param _squareTrackingSpeed 正方形補正走行時の目標速度[mm/s]
   * @param _squareTargetX 正方形を合わせる画像上の目標X座標
   * @param _rotationPid 回頭用PIDゲイン
   * @param _rightPid 右車輪速度制御用PIDゲイン
   * @param _leftPid 左車輪速度制御用PIDゲイン
   * @param _straightAnglePid 通常直進時の角度補正用PIDゲイン
   * @param _squareTrackingPid 正方形による角度補正用PIDゲイン
   */
  RouteFollower(Robot& _robot, const EtRallyMap& _map, const MapData& _mapData,
                double _targetSpeed, double _squareTrackingSpeed, int _squareTargetX,
                const Pid::PidGain& _rotationPid, const Pid::PidGain& _rightPid,
                const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid,
                const Pid::PidGain& _squareTrackingPid);

  /**
   * @brief 経路に従って通常走行する
   * @param route 走行する経路
   */
  void run(const std::vector<RouteState>& route);

  /**
   * @brief 経路に従って走行する
   * @param route 走行する経路
   * @param useSquareCorrection ゲート通過時に正方形補正を使用するか
   */
  void run(const std::vector<RouteState>& route, bool useSquareCorrection);

 private:
  Robot& robot;            // ロボットインスタンス
  const EtRallyMap& map;   // ETラリーの実座標マップ
  const MapData& mapData;  // ゲート情報

  double targetSpeed;  // 通常走行時の目標速度[mm/s]

  double squareTrackingSpeed;  // 正方形補正走行時の目標速度[mm/s]

  int squareTargetX;  // 正方形を合わせる画像上の目標X座標

  Pid::PidGain rotationPid;       // 回頭用PID
  Pid::PidGain rightPid;          // 右車輪速度制御用PID
  Pid::PidGain leftPid;           // 左車輪速度制御用PID
  Pid::PidGain straightAnglePid;  // 通常直進時の角度補正用PID

  Pid::PidGain squareTrackingPid;  // 正方形による角度補正用PID

  /**
   * @brief Directionをロボットの方位角へ変換する
   * @param direction 方向
   * @return 方位角[deg]
   */
  double directionToHeading(Direction direction) const;

  /**
   * @brief 回頭角度を計算する
   * @param from 現在方向
   * @param to 目標方向
   * @return 回頭角度[deg]
   */
  double calculateRotationAngle(Direction from, Direction to) const;

  /**
   * @brief 2地点間の走行距離を計算する
   * @param from 開始地点
   * @param to 終了地点
   * @return 走行距離[mm]
   */
  double calculateDistance(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 指定角度だけ回頭する
   * @param angle 回頭角度[deg]
   */
  void rotate(double angle);

  /**
   * @brief 指定距離を通常直進する
   * @param distance 走行距離[mm]
   */
  void straight(double distance);

  /**
   * @brief 正方形を画像中央に合わせながら走行する
   * @param distance 補正走行距離[mm]
   */
  void straightWithSquareCorrection(double distance);

  /**
   * @brief 正方形を1回検出する
   * @return 正方形を検出した場合true
   */
  bool detectSquare();

  /**
   * @brief 指定区間に対応するゲートを取得する
   * @param from 区間開始地点
   * @param to 区間終了地点
   * @return 対応するゲート。存在しない場合nullptr
   */
  const Gate* findGate(const RouteState& from, const RouteState& to) const;

  /**
   * @brief ゲートが外周ゲートか判定する
   * @param gate ゲート
   * @return 外周ゲートの場合true
   */
  bool isOuterGate(const Gate& gate) const;

  /**
   * @brief 現在地点からゲートまでの距離を計算する
   * @param from 現在地点
   * @param gate ゲート
   * @return ゲートまでの距離[mm]
   */
  double calculateDistanceToGate(const RouteState& from, const Gate& gate) const;

  /**
   * @brief ゲート通過区間を走行する
   * @param from 区間開始地点
   * @param to 区間終了地点
   * @param distance 区間全体の走行距離[mm]
   */
  void runGateSegment(const RouteState& from, const RouteState& to, double distance);
};

#endif  // ROUTE_FOLLOWER_H