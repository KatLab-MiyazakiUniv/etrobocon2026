/**
 * @file   GoalNavigation.h
 * @brief 目標座標まで移動するクラス
 * @author yutaro-1214
 */

#ifndef GOAL_NAVIGATION_H
#define GOAL_NAVIGATION_H

#include "AngleNormalizer.h"
#include "BaseMotion.h"
#include "Pid.h"

class GoalNavigation : public BaseMotion {
 public:
  /**
   * @brief コンストラクタ
   * @param robot ロボット
   * @param continuationCondition 継続条件
   * @param goalX 目標X座標(mm)
   * @param goalY 目標Y座標(mm)
   * @param speed 前進速度
   * @param anglePidGain 方位制御用PIDゲイン
   */
  GoalNavigation(Robot& robot, std::unique_ptr<BaseContinuationCondition> continuationCondition,
                 double goalX, double goalY, double speed, const Pid::PidGain& anglePidGain);

  /**
   * デストラクタ
   */
  ~GoalNavigation();

 protected:
  /**
   * @brief 1周期分の制御を実行
   */
  void executeStep() override;

  /**
   * @brief 動作終了時の処理
   */
  void finish() override;

 private:
  enum class State { ROTATE, STRAIGHT };

  State state;

  double goalX;
  double goalY;
  double speed;

  static constexpr double ANGLE_TOLERANCE = 2.0;

  Pid anglePid;
};

#endif