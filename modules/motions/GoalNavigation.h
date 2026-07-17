/**
 * @file   GoalNavigation.h
 * @brief 目標座標まで移動するクラス
 * @author yutaro-1214
 */

#ifndef GOAL_NAVIGATION_H
#define GOAL_NAVIGATION_H

#include <memory>

#include "AngleNormalizer.h"
#include "BaseContinuationCondition.h"
#include "BaseMotion.h"
#include "Pid.h"
#include "SpeedCalculator.h"

/**
 * @brief 目標座標まで移動する動作クラス
 *
 * 動作は以下の2段階で行う。
 * 1. 目標地点の方向へその場回頭
 * 2. 目標地点まで方位補正しながら直進
 *
 * 終了判定は GoalDistanceCondition が行う。
 */
class GoalNavigation : public BaseMotion {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot Robotクラス
   * @param _continuationCondition 継続条件
   * @param _goalX 目標X座標(mm)
   * @param _goalY 目標Y座標(mm)
   * @param _targetSpeed 目標速度(mm/s)
   * @param _rightPid 右モータ速度PID
   * @param _leftPid 左モータ速度PID
   * @param _anglePidGain 方位制御PID
   */
  GoalNavigation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                 double _goalX, double _goalY, double _targetSpeed, const Pid::PidGain& _rightPid,
                 const Pid::PidGain& _leftPid, const Pid::PidGain& _anglePidGain);

  /**
   * @brief デストラクタ
   */
  ~GoalNavigation();

 protected:
  /**
   * @brief 動作開始前の確認
   * @return true:開始可能 false:開始不可
   */
  bool canStart() override;

  /**
   * @brief 動作開始前の準備
   */
  void prepare() override;

  /**
   * @brief 1周期分の制御
   */
  void executeStep() override;

  /**
   * @brief 動作終了処理
   */
  void finish() override;

 private:
  /**
   * @brief 動作状態
   */
  enum class State {
    ROTATE,   ///< その場回頭
    STRAIGHT  ///< 直進
  };

  State state;

  double goalX;
  double goalY;

  double targetSpeed;

  /// 回頭・方位補正用PID
  Pid anglePid;

  /// 左右速度PID計算
  SpeedCalculator speedCalculator;

  /// 現在向くべき目標角度
  double targetAngle;

  /// 回頭完了と判定する角度誤差(°)
  static constexpr double ANGLE_TOLERANCE = 2.0;
};

#endif