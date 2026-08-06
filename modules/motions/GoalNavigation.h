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
  static constexpr double ROTATION_TOLERANCE = 2.0;   // 回頭完了と判定する角度の許容誤差（度）
  static constexpr double DISTANCE_TOLERANCE = 10.0;  // 目標地点到達と判定する距離の許容誤差（mm）

  Navigator navigator;  // 現在位置から目標地点までの距離と方位角を計算するクラスの参照

  double targetX;      // 目標地点のX座標（mm）
  double targetY;      // 目標地点のY座標（mm）
  double targetSpeed;  // 目標地点へ直進するときの目標速度（mm/s）

  double targetDistance;  // 現在位置から目標地点までの距離（mm）
  double targetHeading;   // 現在位置から目標地点へ向かうための目標方位角（度）

  Pid::PidGain rotationPid;       // 回頭動作で使用する角度制御用PIDゲイン
  Pid::PidGain rightPid;          // 直進動作で使用する右車輪の速度制御用PIDゲイン
  Pid::PidGain leftPid;           // 直進動作で使用する左車輪の速度制御用PIDゲイン
  Pid::PidGain straightAnglePid;  // 直進中の進行方向を維持する角度制御用PIDゲイン
};

#endif  // GOAL_NAVIGATION_H