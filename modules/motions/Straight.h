/**
 * @file   Straight.h
 * @brief  直進動作を実行するクラス
 * @author migaku2645
 */

#ifndef STRAIGHT_H
#define STRAIGHT_H

#include "BaseMotion.h"
#include "SpeedCalculator.h"

class Straight : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @brief Straight を初期化する
   * @param _robot ロボットクラスのインスタンス
   * @param _continuationCondition 動作を継続する条件を判定するクラスのインスタンス
   * @param _targetSpeed 目標速度(mm/秒)
   */

  Straight(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
           double _targetSpeed, Pid::PidGain _rightPid, Pid::PidGain _leftPid);

 protected:
  /**
   * @brief 1周期分の動作を実行する
   */
  void executeStep() override;

  /**
   * @brief 両タイヤモータを停止する
   */
  void finish() override;

 private:
  double targetSpeed;               // 目標速度(mm/ミリ秒)
  SpeedCalculator speedCalculator;  // SpeedCalculatorクラスのインスタンス;
};

#endif