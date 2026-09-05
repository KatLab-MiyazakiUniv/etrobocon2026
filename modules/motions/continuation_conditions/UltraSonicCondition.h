/**
 * @file   UltraSonicCondition.h
 * @brief  超音波距離を基準に動作を継続すべきか判定するクラス
 * @author migaku2645
 */

#ifndef ULTRA_SONIC_CONDITION_H
#define ULTRA_SONIC_CONDITION_H

#include "BaseContinuationCondition.h"
#include "Logger.h"

class UltraSonicCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief メンバ変数 robot を初期化する
   * @param _robot Robot クラスのインスタンスの参照
   * @param _targetDistance 目標超音波距離
   */
  UltraSonicCondition(Robot& _robot, double _targetDistance);

  /**
   * デストラクタ
   * @brief
   */
  ~UltraSonicCondition();

  /**
   * @brief 動作を継続するかを判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

 protected:
  double targetDistance = 0.0;           // 目標超音波距離
  int detectCount = 0;                   // 取得回数
  static constexpr int JUDGE_COUNT = 2;  // 継続条件を終了するために必要な連続取得回数
};

#endif