/**
 * @file   GoalDistanceCondition.cpp
 * @brief 目標座標までの距離を基準に動作継続を判定するクラス
 * @author yutaro-1214
 */

#include "GoalDistanceCondition.h"

GoalDistanceCondition::GoalDistanceCondition(Robot& _robot, double _goalX, double _goalY,
                                             double _tolerance)
  : BaseContinuationCondition(_robot), goalX(_goalX), goalY(_goalY), tolerance(_tolerance)
{
  LOG_CREATE("GoalDistanceCondition");
}

GoalDistanceCondition::~GoalDistanceCondition()
{
  LOG_DESTROY("GoalDistanceCondition");
}

void GoalDistanceCondition::prepare()
{
  // 初期化処理なし
}

bool GoalDistanceCondition::shouldContinue()
{
  double distance = robot.getNavigator().calculateDistance(goalX, goalY);

  // 目標地点まで到達したら終了
  if(distance <= tolerance) {
    return false;
  }

  return true;
}