/**
 * @file   MotionTimeCondition.h
 * @brief  １つの動作の実行時間を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#ifndef MOTION_TIME_CONDITION_H
#define MOTION_TIME_CONDITION_H

#include "BaseContinuationCondition.h"
#include "ClockUtil.h"
#include <cmath>
#include "Logger.h"

class MotionTimeCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief MotionTimeCondition を初期化する
   * @param _robot robotクラスのインスタンスの参照
   * @param _targetTime 目標時間(ms)
   */
  MotionTimeCondition(Robot& _robot, int _targetTime);

  /**
   * デストラクタ
   */
  ~MotionTimeCondition();

  /**
   * @brief 動作開始時の時間を取得して initTime に保存する
   */
  void prepare() override;

  /**
   * @brief 動作開始時からの経過時間が目標時間に達した場合、動作を継続しないと判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

 private:
  int targetTime;  // 目標時間(ms)
  int initTime;    // 開始時の時間(ms)
};

#endif