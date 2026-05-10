/**
 * @file   Straight.h
 * @brief  直進動作を実行するクラス
 * @author migaku2645
 */

#ifndef STRAIGHT_H
#define STRAIGHT_H

#include "BaseMotion.h"
#include "SpeedCalculator.h"
#include "ClockUtil.h"

class Straight : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @brief メンバ変数 Straight を初期化する
   * @param _robot Straight クラスのインスタンスの参照
   * @param _continuationCondition 動作を継続する条件を判定するクラスのインスタンス
   * @param _targetSpeed 目標とするタイヤ走行速度[mm/s]
   */

  Straight(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
           double _targetSpeed);

  /**
   * @brief 1周期分の動作を実行する
   */
  void executeStep() override;

  /**
   * @brief 1周期分の待機を行う (デフォルトは何もしない)
   */
  void wait() override;

 private:
  double targetSpeed;  // 目標速度
  SpeedCalculator
      speedCalculator;  // 目標速度に相当するモーターのPower値を算出するクラスのインスタンス;
};

#endif