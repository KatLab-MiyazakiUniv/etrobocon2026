/**
 * @file   DistanceCondition.cpp
 * @brief  直進動作の継続条件を判定するクラス
 * @author migaku2645
 */

#ifndef DISTANCE_CONDITION_H
#define DISTANCE_CONDITION_H

#include "BaseContinuationCondition.h"
#include "Mileage.h"
#include <cmath>

class DistanceCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief メンバ変数 DistanceCondition を初期化する
   * @param _robot DistanceCondition クラスのインスタンスの参照
   * @param _targetDistance 目標距離
   */
  DistanceCondition(Robot& _robot, double _targetDistance);

  /**
   * @brief 継続条件の事前準備を行う (デフォルトは何もしない)
   */
  void prepare() override;

  /**
   * @brief 動作を継続するかを判定する
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

 private:
  double targetDistance;   // 目標距離
  double initDistance;     // 開始時の累計走行距離
};

#endif