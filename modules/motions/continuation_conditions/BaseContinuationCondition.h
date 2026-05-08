/**
 * @file   BaseContinuationCondition.h
 * @brief  動作を継続するかを判定するクラスの基底クラス
 * @author takuchi17
 */

#ifndef BASE_CONTINUATION_CONDITION_H
#define BASE_CONTINUATION_CONDITION_H

#include "Robot.h"

class BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief メンバ変数 robot を初期化する
   * @param _robot Robot クラスのインスタンスの参照
   */
  BaseContinuationCondition(Robot& _robot);

  /**
   * デストラクタ
   * @brief 基底クラス経由で派生クラスのインスタンスを安全に破棄できるようにする
   */
  virtual ~BaseContinuationCondition() = default;

  /**
   * @brief 継続条件の事前準備を行う (デフォルトは何もしない)
   */
  virtual void prepare();

  /**
   * @brief 動作を継続するかを判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  virtual bool shouldContinue() = 0;

  /**
   * @brief 継続条件の内部状態を更新する (デフォルトは何もしない)
   */
  virtual void update();

 protected:
  Robot& robot;  // Robot インスタンスの参照
};

#endif