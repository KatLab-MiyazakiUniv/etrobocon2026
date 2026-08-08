/**
 * @file   GoalNavigation.h
 * @brief  目標地点への回頭と直進を実行するクラス
 * @author yutaro-1214
 */

#ifndef GOAL_NAVIGATION_H
#define GOAL_NAVIGATION_H

#include <memory>

#include "BaseMotion.h"
#include "Navigator.h"
#include "AbsoluteRotation.h"
#include "AbsoluteAngleCondition.h"
#include "Straight.h"
#include "DistanceCondition.h"
#include "Pid.h"
#include "Logger.h"

class GoalNavigation : public BaseMotion {
public:
  /**
   * @brief コンストラクタ
   *
   * @param _robot Robotクラスのインスタンス
   * @param _condition GoalNavigation全体の継続条件
   * @param _targetX 目標地点X座標[mm]
   * @param _targetY 目標地点Y座標[mm]
   * @param _targetSpeed 直進時の目標速度[mm/s]
   * @param _rotationPid 回頭制御用PID
   * @param _rightPid 右タイヤ速度制御用PID
   * @param _leftPid 左タイヤ速度制御用PID
   * @param _straightAnglePid 直進時の角度制御用PID
   */
  GoalNavigation(
      Robot& _robot,
      std::unique_ptr<BaseContinuationCondition> _condition,
      double _targetX,
      double _targetY,
      double _targetSpeed,
      const Pid::PidGain& _rotationPid,
      const Pid::PidGain& _rightPid,
      const Pid::PidGain& _leftPid,
      const Pid::PidGain& _straightAnglePid);

  ~GoalNavigation() override;

protected:
  /**
   * @brief 目標地点までの距離と方位角を計算する
   */
  void prepare() override;

  /**
   * @brief 目標地点への回頭と直進を実行する
   */
  void executeStep() override;

  /**
   * @brief 動作終了時にモーターを停止する
   */
  void finish() override;

private:
  /**
   * @brief 回頭完了と判定する角度誤差
   */
  static constexpr double ROTATION_TOLERANCE = 2.0;

  /**
   * @brief 目標地点到達と判定する距離誤差[mm]
   */
  static constexpr double DISTANCE_TOLERANCE = 10.0;

  /**
   * @brief Navigator
   */
  Navigator& navigator;

  /**
   * @brief 目標地点
   */
  double targetX;
  double targetY;

  /**
   * @brief 走行速度[mm/s]
   */
  double targetSpeed;

  /**
   * @brief 現在位置から目標地点までの距離[mm]
   */
  double targetDistance;

  /**
   * @brief 現在位置から目標地点への方位角[deg]
   */
  double targetHeading;

  /**
   * @brief PID
   */
  Pid::PidGain rotationPid;
  Pid::PidGain rightPid;
  Pid::PidGain leftPid;
  Pid::PidGain straightAnglePid;
};

#endif  // GOAL_NAVIGATION_H