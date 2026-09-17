/**
 * @file ETZumoFinish.h
 * @brief ET相撲からフィニッシュまでの子動作を順に実行する複合動作
 */

#ifndef ET_ZUMO_FINISH_H
#define ET_ZUMO_FINISH_H

#include "BaseMotion.h"
#include "ProjectedMileage.h"
#include <memory>
#include <vector>

class ETZumoFinish : public BaseMotion {
 public:
  /**
   * @brief ET相撲からフィニッシュまでの動作を初期化する
   * @param robot Robotクラスのインスタンス
   * @param condition 複合動作の継続条件
   * @param children 実行順に保持する子動作
   * @param sharedMileage 子動作と共有する基準方向の距離計測
   */
  ETZumoFinish(Robot& robot, std::unique_ptr<BaseContinuationCondition> condition,
               std::vector<std::unique_ptr<BaseMotion>> children,
               std::shared_ptr<ProjectedMileage> sharedMileage
               = std::make_shared<ProjectedMileage>());

 protected:
  /**
   * @brief 実行完了状態を初期化し、方位を0度にリセットして距離計測の起点を設定する
   */
  void prepare() override;

  /**
   * @brief 子動作を一度だけ順に実行する
   * @details 距離計測が無効になった場合、残りの子動作を中止する
   */
  void executeStep() override;

  /**
   * @brief モーターを停止し、水平・垂直方向の移動距離を記録する
   */
  void finish() override;

 private:
  std::vector<std::unique_ptr<BaseMotion>> motions;  // 実行順に保持する子動作
  std::shared_ptr<ProjectedMileage> mileage;         // 子動作と共有する基準方向の距離計測
  bool completed = false;                            // 子動作の実行が完了または中止した場合はtrue
};
#endif
