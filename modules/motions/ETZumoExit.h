/**
 * @file ETZumoExit.h
 * @brief 基準方向の距離を使ってETZumoエリアを突破する複合動作
 * @author miyahara046
 */
/**
 * @file ETZumoExit.h
 * @brief 追尾・回頭・直進を順番に実行する複合動作
 */

#ifndef ET_ZUMO_EXIT_H
#define ET_ZUMO_EXIT_H

#include <memory>
#include <vector>

#include "BaseMotion.h"

class ETZumoExit : public BaseMotion {
 public:
  /**
   * @brief 追尾、回頭、直進を順番に実行する複合動作を初期化する
   * @param _robot Robotクラスのインスタンス
   * @param _continuationCondition 複合動作の継続条件
   * @param _motions 追尾、回頭、直進の順に実行する動作
   */
  ETZumoExit(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
             std::vector<std::unique_ptr<BaseMotion>> _motions);

  ~ETZumoExit() override;

 protected:
  /**
   * @brief 3つの子動作が設定されているか確認する
   */
  bool canStart() override;

  /**
   * @brief 追尾、回頭、直進を順番に実行する
   */
  void executeStep() override;

  /**
   * @brief モーターを停止する
   */
  void finish() override;

 private:
  std::vector<std::unique_ptr<BaseMotion>> motions;
};

#endif