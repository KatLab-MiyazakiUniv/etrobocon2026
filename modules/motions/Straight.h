/**
 * @file   Straight.h
 * @brief  直進動作を実行するクラス
 * @author migaku2645
 */

#ifndef STRAIGHT_H
#define STRAIGHT_H

#include "BaseMotion.h"
#include "SpeedCalculator.h"
#include "Pid.h"
#include "AngleNormalizer.h"

class Straight : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @brief Straight を初期化する
   * @param _robot ロボットクラスのインスタンス
   * @param _continuationCondition 動作を継続する条件を判定するクラスのインスタンス
   * @param _targetSpeed 目標速度(mm/秒)
   * @param _rightPid 右タイヤのPIDゲイン
   * @param _leftPid 左タイヤのPIDゲイン
   * @param _anglePidGain 角度制御用PIDゲイン
   * @param _shouldUseIMU IMUを使用するかどうか
   */

  Straight(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
           double _targetSpeed, const Pid::PidGain& _rightPid, const Pid::PidGain& _leftPid,
           const Pid::PidGain& _anglePidGain, bool _shouldUseIMU);

 protected:
  /**
   * @brief 動作を開始できるかどうかを判定する
   */
  bool canStart() override;

  /**
   * @brief 動作を開始する前に必要な準備を行う
   */
  void prepare() override;

  /**
   * @brief 1周期分の動作を実行する
   */
  void executeStep() override;

  /**
   * @brief 両タイヤモータを停止する
   */
  void finish() override;

 private:
  double targetSpeed;               // 目標速度(mm/秒)
  double targetAngle;               // 目標角度(度)
  SpeedCalculator speedCalculator;  // SpeedCalculatorクラスのインスタンス
  Pid anglePid;                     // 角度制御用PID
  bool shouldUseIMU;                // IMUを使用するかどうか
};

#endif