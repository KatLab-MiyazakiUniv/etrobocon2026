/**
 * @file   UltraSonicCondition.cpp
 * @brief  目標色を基準に動作を継続すべきかを判定するクラス
 * @author migaku2645
 */

#include "UltraSonicCondition.h"

UltraSonicCondition::UltraSonicCondition(Robot& _robot, double _targetDistance)
  : BaseContinuationCondition(_robot), targetDistance(_targetDistance)
{
  LOG_CREATE("UltraSonicCondition");
}

UltraSonicCondition::~UltraSonicCondition()
{
  LOG_DESTROY("UltraSonicCondition");
}

void UltraSonicCondition::prepare()
{
  // 初期値を代入
  initDistance = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                           robot.getWheelMotorControllerInstance().getLeftCount());

  // 超音波カウントを初期化
  detectCount = 0;
}
bool UltraSonicCondition::shouldContinue()
{
  // 超音波センサーの距離を取得
  double detectDistance = robot.getUltrasonicSensorInstance().getDistance();

  // 現在の超音波距離が指定した超音波距離と一致していればカウント増加、違えばリセット
  if(targetSpeed > 0.0) {
    // 前進時、指定距離以下かつ正常値のときカウント増加
    if(detectDistance <= targetDistance && detectDistance > 0.0) {
      detectCount++;
    } else {
      detectCount = 0;
    }
  }
  //  else {
  //   // 後退時、指定距離以上かつ正常値のときカウント増加
  //   if(detectDistance >= targetDistance && detectDistance > 0.0) {
  //     detectCount++;
  //   } else {
  //     detectCount = 0;
  //   }
  // }

  // 指定された色をJUDGE_COUNT回連続で取得したときモータが止まる
  if(detectCount >= JUDGE_COUNT) {
    Logger::info("Target distance detected");
    return false;
  }

  return true;
}