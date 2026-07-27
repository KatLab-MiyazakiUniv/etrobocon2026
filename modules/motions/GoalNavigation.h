/**
 * @file   GoalNavigation.h
 * @brief  目標地点への回頭と直進を実行するクラス
 * @author yutaro-1214
 */

#ifndef GOAL_NAVIGATION_H
#define GOAL_NAVIGATION_H

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
   * @param _robot ロボットクラスのインスタンス
   * @param _Condition GoalNavigation全体の継続条件
   * @param _targetX 目標地点のX座標[mm]
   * @param _targetY 目標地点のY座標[mm]
   * @param _targetSpeed 直進時の目標速度[mm/s]
   * @param _rotationPid 回頭制御用PIDゲイン
   * @param _rightPid 右タイヤ速度制御用PIDゲイン
   * @param _leftPid 左タイヤ速度制御用PIDゲイン
   * @param _straightAnglePid 直進時の角度制御用PIDゲイン
   */
  GoalNavigation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _Condition,
                 double _targetX, double _targetY, double _targetSpeed,
                 const Pid::PidGain& _rotationPid, const Pid::PidGain& _rightPid,
                 const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid);

  ~GoalNavigation();

 protected:
  /**
   * @brief 目標地点までの距離と方位角を計算する
   */
  void prepare() override;

  /**
   * @brief 回頭と直進を実行する
   */
  void executeStep() override;

  /**
   * @brief 動作終了時にモーターを停止する
   */
  void finish() override;

 private:
  Navigator& navigator;

  double targetX;
  double targetY;
  double targetSpeed;

  double targetDistance;
  double targetHeading;

  Pid::PidGain rotationPid;
  Pid::PidGain rightPid;
  Pid::PidGain leftPid;
  Pid::PidGain straightAnglePid;
};

#endif  // GOAL_NAVIGATION_H