/**
 * @file ETZumoExit.h
 * @brief 基準方向の距離を使ってETZumoエリアを突破する複合動作
 * @author miyahara046
 */

#ifndef ET_ZUMO_EXIT_H
#define ET_ZUMO_EXIT_H

#include <vector>
#include "BaseMotion.h"
#include "ProjectedMileage.h"
#include "Mileage.h"

class ETZumoExit : public BaseMotion {
 public:
  /**
   * @brief 基準設定からETZumoエリア突破までの動作を初期化する
   * @param _robot Robotクラスのインスタンス
   * @param _continuationCondition 複合動作の継続条件
   * @param _mileage 子動作と共有する基準方向の距離計測
   * @param _motions 追尾、回頭、直進の順に実行する動作
   */
  ETZumoExit(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
             std::shared_ptr<ProjectedMileage> _mileage,
             std::vector<std::unique_ptr<BaseMotion>> _motions);

  /**
   * @brief デストラクタ
   */
  ~ETZumoExit() override;

 protected:
  /**
   * @brief 距離計測と3つの子動作が設定されているか確認する
   * @return true/開始可能、false/設定不備
   */
  bool canStart() override;

  /**
   * @brief 方位を0度にリセットし、距離計測の起点を設定する
   */
  void prepare() override;
  /**
   * @brief 追尾、回頭、直進を順に実行する
   */
  void executeStep() override;
  /**
   * @brief モーターを停止し、基準方向の移動距離を記録する
   */
  void finish() override;

 private:
  std::shared_ptr<ProjectedMileage> mileage;         // 基準方向の距離計測
  std::vector<std::unique_ptr<BaseMotion>> motions;  // 実行順に保持する子動作
};

#endif
