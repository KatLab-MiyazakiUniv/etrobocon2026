/**
 * @file   EtRallyCompletionCondition.h
 * @brief  ETラリー全体の完了状態に基づいて動作の継続を判定するクラス
 */

#ifndef ET_RALLY_COMPLETION_CONDITION_H
#define ET_RALLY_COMPLETION_CONDITION_H

#include <functional>
#include "BaseContinuationCondition.h"

/**
 * @brief ETラリーの完了状態に基づいて動作の継続を判定するクラス
 */
class EtRallyCompletionCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief EtRallyCompletionConditionを初期化する
   * @param _robot ロボットクラスのインスタンスの参照
   * @param _notCompleted ETラリーが未完了かどうかを判定する関数
   */
  EtRallyCompletionCondition(Robot& _robot, std::function<bool()> _notCompleted);

  /**
   * @brief ETラリーが未完了の場合、動作を継続すると判定する
   * @return true 動作を継続する、false 動作を継続しない
   */
  bool shouldContinue() override;

 private:
  std::function<bool()> notCompleted;  // ETラリーが未完了かどうかを判定する関数
};

#endif  // ET_RALLY_COMPLETION_CONDITION_H