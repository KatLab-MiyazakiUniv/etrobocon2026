/**
 * @file   MotionTimeContinuationCondition.h
 * @brief  目標時間を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#ifndef MOTION_TIME_CONTINUATION_CONDITION_H
#define MOTION_TIME_CONTINUATION_CONDITION_H

#include "BaseContinuationCondition.h"
#include "ClockUtil.h"
#include <cmath>
#include "Logger.h"

class MotionTimeContinuationCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief MotionTimeContinuationCondition を初期化する
   * @param _robot robotクラスのインスタンスの参照
   * @param _targetTime 目標時間(ms)
   * @param _initTime 開始時の時間(ms)
   */
  MotionTimeContinuationCondition(Robot& _robot, double _targetTime);

  /**
   * デストラクタ
   */
  ~MotionTimeContinuationCondition();

  /**
   * @brief 開始時の時間を取得して initTime に保存する
   */
  void prepare() override;

  /**
   * @brief 開始時からの経過時間が目標時間に達した場合、動作を継続しないと判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

 private:
  double targetTime;  // 目標時間(ms)
  double initTime;    // 開始時の時間(ms)
};

#endif