/**
 * @file   CompoundCondition.h
 * @brief  複数の継続条件を組み合わせて動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#ifndef COMPOUND_CONTINUATION_CONDITION_H
#define COMPOUND_CONTINUATION_CONDITION_H

#include "BaseContinuationCondition.h"
#include "Logger.h"
#include <memory>

class CompoundCondition : public BaseContinuationCondition {
 public:
  /**
   * @brief 論理演算子のenumクラス
   * @param AND &&
   * @param OR ||
   */
  enum class LogicalOperator { AND, OR };

  /**
   * コンストラクタ
   * @brief CompoundCondition を初期化する
   * @param _robot robotクラスのインスタンスの参照
   * @param _firstContinuationCondition 最初の継続条件
   * @param _secondContinuationCondition 次の継続条件
   */
  CompoundCondition(Robot& _robot,
                    std::unique_ptr<BaseContinuationCondition> _firstContinuationCondition,
                    std::unique_ptr<BaseContinuationCondition> _secondContinuationCondition,
                    LogicalOperator _operator);

  /**
   * デストラクタ
   */
  ~CompoundCondition();

  /**
   * @brief 各条件判定クラスの事前準備を行う
   */
  void prepare() override;

  /**
   * @brief 各条件判定クラスと論理演算子から動作を継続すべきかを判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

 private:
  std::unique_ptr<BaseContinuationCondition>
      firstContinuationCondition;  // 条件判定クラス1のポインタ
  std::unique_ptr<BaseContinuationCondition>
      secondContinuationCondition;  // 条件判定クラス2のポインタ
  LogicalOperator logicalOperator;  // 論理演算子
};

#endif