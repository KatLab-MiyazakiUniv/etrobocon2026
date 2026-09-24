/**
 * @file   ProjectedDistanceCondition.h
 * @brief  共通座標を更新し、子条件または目標座標で動作の継続を判定するクラス
 * @author miyahara046
 */

#ifndef PROJECTED_DISTANCE_CONDITION_H
#define PROJECTED_DISTANCE_CONDITION_H

#include "BaseContinuationCondition.h"
#include "ProjectedMileage.h"
#include <memory>

class ProjectedDistanceCondition : public BaseContinuationCondition {
 public:
  // 基準方向を水平軸、基準方向の+90度方向を垂直軸とする
  enum class Axis { HORIZONTAL, VERTICAL };

  /**
   * @brief 共通座標を更新し、子条件で動作の継続を判定するコンストラクタ
   * @param robot 制御対象
   * @param mileage 共通座標の計算に使用する距離計測インスタンス
   * @param child 動作の継続を判定する子条件
   */
  ProjectedDistanceCondition(Robot& robot, std::shared_ptr<ProjectedMileage> mileage,
                             std::unique_ptr<BaseContinuationCondition> child);

  /**
   * @brief 指定した軸の目標座標で動作の継続を判定するコンストラクタ
   * @param robot 制御対象
   * @param mileage 共通座標の計算に使用する距離計測インスタンス
   * @param axis 判定に使用する軸
   * @param target 符号付きの目標座標(mm)
   */
  ProjectedDistanceCondition(Robot& robot, std::shared_ptr<ProjectedMileage> mileage, Axis axis,
                             double target);

  /**
   * @brief デストラクタ
   */
  ~ProjectedDistanceCondition();

  /**
   * @brief 子条件がある場合、判定開始前の準備処理を行う
   */
  void prepare() override;

  /**
   * @brief 共通座標を更新し、子条件または目標座標で動作の継続を判定する
   * @details 正の目標は以上、負の目標は以下になったら終了する。距離計測が無効な場合も終了する。
   * @return true/動作を継続する、false/動作を継続しない
   */
  bool shouldContinue() override;

 private:
  std::shared_ptr<ProjectedMileage> mileage;         // 共通座標の距離計測インスタンス
  std::unique_ptr<BaseContinuationCondition> child;  // 動作の継続を判定する子条件
  Axis axis = Axis::HORIZONTAL;                      // 判定に使用する軸
  double target = 0.0;                               // 符号付きの目標座標(mm)
  bool useTarget = false;                            // 目標座標で判定する場合はtrue
};
#endif
