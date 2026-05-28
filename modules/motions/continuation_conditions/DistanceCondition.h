/**
 * @file   DistanceCondition.cpp
 * @brief  目標距離を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#ifndef DISTANCE_CONDITION_H
#define DISTANCE_CONDITION_H

#include "BaseContinuationCondition.h"
#include "Mileage.h"
#include <cmath>
#include "Logger.h"

class DistanceCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @brief DistanceCondition を初期化する
   * @param _robot robotクラスのインスタンスの参照
   * @param _targetDistance 目標距離(mm)
   */
  DistanceCondition(Robot& _robot, double _targetDistance);

  /**
   * デストラクタ
   */
  ~DistanceCondition();

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
  double targetDistance;  // 目標距離(mm)
  double initDistance;    // 開始時の累計走行距離(mm)
};

#endif