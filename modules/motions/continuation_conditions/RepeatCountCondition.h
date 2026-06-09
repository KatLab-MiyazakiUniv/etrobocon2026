/**
 * @file   RepeatCountCondition.h
 * @brief  目標繰り返し回数を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#ifndef REPEAT_COUNT_CONTINUATION_CONDITION_H
#define REPEAT_COUNT_CONTINUATION_CONDITION_H

#include "BaseContinuationCondition.h"
#include <cmath>
#include "Logger.h"

class RepeatCountCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief RepeatCountCondition を初期化する
   * @param _robot robotクラスのインスタンスの参照
   * @param _targetRepeats 目標繰り返し回数
   */
  RepeatCountCondition(Robot& _robot, int _targetRepeats);

  /**
   * デストラクタ
   */
  ~RepeatCountCondition();

  /**
   * @brief 開始時からの繰り返し回数が目標繰り返し回数に達した場合、動作を継続しないと判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

 private:
  int targetRepeats;       // 目標繰り返し回数
  int currentRepeats = 0;  // 現在の繰り返し回数
};

#endif