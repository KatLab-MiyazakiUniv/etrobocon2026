/**
 * @file   GoalDistanceCondition.h
 * @brief 目標座標までの距離を基準に動作継続を判定するクラス
 * @author yutaro-1214
 */

#ifndef GOAL_DISTANCE_CONDITION_H
#define GOAL_DISTANCE_CONDITION_H

#include "BaseContinuationCondition.h"
#include "Logger.h"

class GoalDistanceCondition : public BaseContinuationCondition {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot Robotクラスのインスタンス
   * @param _goalX 目標X座標(mm)
   * @param _goalY 目標Y座標(mm)
   * @param _tolerance 到達判定距離(mm)
   */
  GoalDistanceCondition(Robot& _robot, double _goalX, double _goalY, double _tolerance);

  /**
   * @brief デストラクタ
   */
  ~GoalDistanceCondition();

  /**
   * @brief 動作開始前の準備
   *
   * BaseContinuationConditionと同様、今回は何もしない。
   */
  void prepare() override;

  /**
   * @brief 動作を継続するか判定する
   * @return true:継続 false:終了
   */
  bool shouldContinue() override;

 private:
  double goalX;      ///< 目標X座標(mm)
  double goalY;      ///< 目標Y座標(mm)
  double tolerance;  ///< 到達判定距離(mm)
};

#endif