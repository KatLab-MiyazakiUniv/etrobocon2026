/**
 * @file   BaseContinuationCondition.h
 * @brief  動作を継続するかを判定するクラスの基底クラス
 * @author takuchi17
 */

#ifndef ULTRA_SONIC_CONDITION_H
#define ULTRA_SONIC_CONDITION_H

#include "BaseContinuationCondition.h"
#include "Mileage.h"
#include "Logger.h"
#include ""

class UltraSonicCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief メンバ変数 robot を初期化する
   * @param _robot Robot クラスのインスタンスの参照
   */
  UltraSonicCondition(Robot& _robot, double _targetDistance);

  /**
   * デストラクタ
   * @brief 基底クラス経由で派生クラスのインスタンスを安全に破棄できるようにする
   */
  ~UltraSonicCondition();

  /**
   * @brief 継続条件の事前準備を行う (デフォルトは何もしない)
   */
  void prepare() override;

  /**
   * @brief 動作を継続するかを判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  virtual bool shouldContinue() = 0;

 protected:
  double targetDistance = 0;
  double initDistance = 0;
  double detectCount;
  static constexpr int JUDGE_COUNT = 2;  // 継続条件を修了するために必要な連続取得回数
};

#endif