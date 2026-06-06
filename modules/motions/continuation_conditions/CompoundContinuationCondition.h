/**
 * @file   CompoundContinuationCondition.h
 * @brief  複数の継続条件を組み合わせて動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#ifndef COMPOUND_CONTINUATION_CONDITION_H
#define COMPOUND_CONTINUATION_CONDITION_H

#include "BaseContinuationCondition.h"
#include "Logger.h"
#include <memory>

class CompoundContinuationCondition : public BaseContinuationCondition {
 public:
  enum class LogicalOperator { AND, OR };

  /**
   * コンストラクタ
   * @brief CompoundContinuationCondition を初期化する
   * @param _robot robotクラスのインスタンスの参照
   * @param _firstContinuationCondition 最初の継続条件
   * @param _secondContinuationCondition 次の継続条件
   */
  CompoundContinuationCondition(
      Robot& _robot, std::unique_ptr<BaseContinuationCondition> _firstContinuationCondition,
      std::unique_ptr<BaseContinuationCondition> _secondContinuationCondition,
      LogicalOperator _operator);

  /**
   * デストラクタ
   */
  ~CompoundContinuationCondition();

  /**
   * @brief 開始時の累計走行距離を取得して initDistance に保存する
   */
  void prepare() override;

  /**
   * @brief 開始時からの移動距離が目標距離に達した場合、動作を継続しないと判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

 private:
  std::unique_ptr<BaseContinuationCondition> firstContinuationCondition;
  std::unique_ptr<BaseContinuationCondition> secondContinuationCondition;
  LogicalOperator logicalOperator;
};

#endif