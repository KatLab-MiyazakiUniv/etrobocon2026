/**
 * @file   RouteFollower.h
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#ifndef ROUTE_FOLLOWER_H
#define ROUTE_FOLLOWER_H

#include <cmath>
#include <memory>
#include <vector>

#include "AngleNormalizer.h"
#include "ClockUtil.h"
#include "DistanceCondition.h"
#include "EtRallyMap.h"
#include "Logger.h"
#include "Pid.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "Robot.h"
#include "RouteTypes.h"
#include "SocketProtocol.h"
#include "SquareAngleAdjustment.h"
#include "Straight.h"
#include "SystemInfo.h"

/**
 * @brief 経路探索結果に従って走行するクラス
 */
class RouteFollower {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot ロボット
   * @param _map ETラリーマップ
   * @param _targetSpeed 直進速度
   * @param _rotationPid 通常回頭PID
   * @param _squareRotationPid 正方形補正回頭PID
   * @param _straightAnglePid 直進角度PID
   * @param _straightDeadbandRate 直進デッドバンド率
   * @param _straightMaxoutRate 直進マックスアウト率
   */
  RouteFollower(Robot& _robot, const EtRallyMap& _map, double _targetSpeed,
                const Pid::PidGain& _rotationPid, const Pid::PidGain& _squareRotationPid,
                const Pid::PidGain& _straightAnglePid, double _straightDeadbandRate,
                double _straightMaxoutRate);

  /**
   * @brief 経路探索結果に従って走行する
   * @param route 走行する経路
   */
  void run(const std::vector<RouteState>& route);

 private:
  Robot& robot;                    // ロボット本体
  const EtRallyMap& map;           // ETラリーのマップ情報
  double targetSpeed;              // 目標走行速度
  Pid::PidGain rotationPid;        // 通常回頭用PIDゲイン
  Pid::PidGain squareRotationPid;  // 正方形補正回頭用PIDゲイン
  Pid::PidGain straightAnglePid;   // 直進角度PIDゲイン
  double straightDeadbandRate;     // 直進デッドバンド率
  double straightMaxoutRate;       // 直進マックスアウト率

  /**
   * @brief Directionを角度へ変換する
   * @param direction 変換する方向
   * @return Directionに対応する角度[deg]
   */
  double directionToHeading(Direction direction) const;

  /**
   * @brief 2方向間で必要な回頭角度を計算する
   * @param from 現在方向
   * @param to 目標方向
   * @return 必要な回頭角度[deg]
   */
  double calculateRotationAngle(Direction from, Direction to) const;

  /**
   * @brief 2地点間の距離を計算する
   * @param from 区間開始状態
   * @param to 区間終了状態
   * @return 2地点間の距離[mm]
   */
  double calculateDistance(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 通常回頭を行う
   * @param angle 回頭角度[deg]
   */
  void rotate(double angle);

  /**
   * @brief 正方形補正用の回頭を行う
   * @param angle 回頭角度[deg]
   */
  void rotateForSquare(double angle);

  /**
   * @brief 指定距離を直進する
   * @param distance 直進距離[mm]
   */
  void straight(double distance);

  /**
   * @brief 指定距離を後退する
   * @param distance 後退距離[mm]
   */
  void backward(double distance);

  /**
   * @brief 正方形を検出して補正情報を取得する
   * @param result 正方形補正結果
   * @return true 正方形検出成功
   * @return false 正方形検出失敗
   */
  bool detectSquare(SquareAngleAdjustment::Result& result);

  /**
   * @brief ゲートを含む区間を走行する
   * @param from 区間開始状態
   * @param to 区間終了状態
   * @param distance 区間距離[mm]
   * @param rotatedAtSegmentStart 区間開始時に回頭した場合はtrue
   */
  void runGateSegment(const RouteState& from, const RouteState& to, double distance,
                      bool rotatedAtSegmentStart);

  /**
   * @brief 区間に存在するゲートを取得する
   * @param from 区間開始状態
   * @param to 区間終了状態
   * @return 区間内に存在するゲートへのポインタ
   * @return nullptr ゲートが存在しない場合
   */
  const Gate* findGate(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 外周ゲートか判定する
   * @param gate 判定するゲート
   * @return true 外周ゲート
   * @return false 内側ゲート
   */
  bool isOuterGate(const Gate& gate) const;

  /**
   * @brief 区間開始位置からゲート中央までの距離を計算する
   * @param from 区間開始状態
   * @param gate 距離を計算するゲート
   * @return 区間開始位置からゲート中央までの距離[mm]
   */
  double calculateDistanceToGate(const RouteState& from, const Gate& gate) const;
};

#endif  // ROUTE_FOLLOWER_H