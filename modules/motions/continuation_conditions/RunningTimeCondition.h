/**
 * @file   RunningTimeCondition.h
 * @brief  目標時間を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#ifndef RUNNING_TIME_CONTINUATION_CONDITION_H
#define RUNNING_TIME_CONTINUATION_CONDITION_H

#include "BaseContinuationCondition.h"
#include "ClockUtil.h"
#include <cmath>
#include "Logger.h"

class RunningTimeCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief RunningTimeCondition を初期化する
   * @param _robot robotクラスのインスタンスの参照
   * @param _targetTime 目標時間(ms)
   * @param _initTime 開始時の時間(ms)
   */
  RunningTimeCondition(Robot& _robot, int _targetTime);

  /**
   * デストラクタ
   */
  ~RunningTimeCondition();

  /**
   * @brief 開始時からの経過時間が目標時間に達した場合、動作を継続しないと判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

 private:
  int targetTime;  // 目標時間(ms)
};

#endif