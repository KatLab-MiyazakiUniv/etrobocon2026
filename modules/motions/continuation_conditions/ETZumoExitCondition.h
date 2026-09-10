/**
 * @file ETZumoExitCondition.h
 * @brief 基準方向の累積距離と子動作の継続条件で終了を判定するクラス
 * @author miyahara046
 */

#ifndef ET_ZUMO_EXIT_CONDITION_H
#define ET_ZUMO_EXIT_CONDITION_H

#include <memory>
#include "BaseContinuationCondition.h"
#include "ProjectedMileage.h"
#include "Mileage.h"

class ETZumoExitCondition : public BaseContinuationCondition {
 public:
  /**
   * @brief 子動作の継続条件と共通の距離計測を初期化する
   * @param _robot Robotクラスのインスタンス
   * @param _mileage 追尾・回頭・直進で共有する距離計測（nullptrは不可）
   * @param _targetDistance 基準方向の目標距離[mm]（有限の正の値）
   * @param _continuationCondition 子動作の継続条件（直進時はnullptrで省略）
   */
  ETZumoExitCondition(Robot& _robot, std::shared_ptr<ProjectedMileage> _mileage,
                      double _targetDistance,
                      std::unique_ptr<BaseContinuationCondition> _continuationCondition = nullptr,
                      bool _requireBoth = false);

  /**
   * @brief デストラクタ
   */
  ~ETZumoExitCondition() override;

  /**
   * @brief 子動作の継続条件を初期化する
   * @details 共有する積算距離はリセットせず、前の子動作から引き継ぐ。
   */
  void prepare() override;

  /**
   * @brief 距離を更新し、目標距離と子動作の条件から継続を判定する
   * @return true/動作を継続する、false/目標到達・子動作終了・計測異常
   */
  bool shouldContinue() override;

 private:
  std::shared_ptr<ProjectedMileage> mileage;  // 基準方向の距離計測
  bool requireBoth;       // trueなら距離到達後に子条件の成立を待つ
  double targetDistance;  // 基準方向の目標距離[mm]
  std::unique_ptr<BaseContinuationCondition> continuationCondition;  // 子動作の継続条件
};

#endif
