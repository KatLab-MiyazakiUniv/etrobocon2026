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
   * @param _targetSpeed 直進速度
   * @param _rotationPid 通常回頭PID
   * @param _squareRotationPid 正方形補正回頭PID
   * @param _rightPid 右モーターPID
   * @param _leftPid 左モーターPID
   * @param _straightAnglePid 直進角度PID
   * @param _straightDeadbandRate 直進デッドバンド
　 * @param _straightMaxoutRate 直進マックスアウト
   */
  RouteFollower(Robot& _robot, const EtRallyMap& _map, double _targetSpeed,
                const Pid::PidGain& _rotationPid, const Pid::PidGain& _squareRotationPid,
                const Pid::PidGain& _rightPid, const Pid::PidGain& _leftPid,
                const Pid::PidGain& _straightAnglePid, double _straightDeadbandRate,
                double _straightMaxoutRate);

  /**
   * @brief 経路を走行する
   */
  void run(const std::vector<RouteState>& route);

 private:
  Robot& robot;                    // ロボット本体
  const EtRallyMap& map;           // ETラリーのマップ情報
  double targetSpeed;              // 目標走行速度
  Pid::PidGain rotationPid;        // 回頭用PIDゲイン
  Pid::PidGain squareRotationPid;  // 正方形補正用回頭PIDゲイン
  Pid::PidGain rightPid;           // 右モーター用PIDゲイン
  Pid::PidGain leftPid;            // 左モーター用PIDゲイン
  Pid::PidGain straightAnglePid;   // 直進角度PIDゲイン
  double straightDeadbandRate;     // 直進デッドバンド率
  double straightMaxoutRate;       // 直進マックスアウト率

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
   * @param from 区間の開始位置
   * @param to 区間の終了位置
   * @return 区間内に存在するゲートへのポインタ。存在しない場合はnullptr
   */
  const Gate* findGate(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 外周ゲートか判定する
   * @param gate 判定するゲート
   * @return 外周ゲートの場合はtrue、それ以外はfalse
   */
  bool isOuterGate(const Gate& gate) const;

  /**
   * @brief 区間開始位置からゲート中央までの距離を計算する
   * @param from 区間の開始位置
   * @param gate 距離を計算するゲート
   * @return 区間開始位置からゲート中央までの距離[mm]
   */
  double calculateDistanceToGate(const RouteState& from, const Gate& gate) const;
};

#endif  // ROUTE_FOLLOWER_H