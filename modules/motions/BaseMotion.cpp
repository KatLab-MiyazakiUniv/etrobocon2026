/**
 * @file   BaseMotion.cpp
 * @brief  動作の実行処理を共通化するための基底クラス
 * @author takuchi17
 */

#include "BaseMotion.h"

#include <utility>

BaseMotion::BaseMotion(Robot& _robot,
                       std::unique_ptr<BaseContinuationCondition> _continuationCondition)
  : robot(_robot), continuationCondition(std::move(_continuationCondition))
{
}

void BaseMotion::run()
{
  // 動作を開始可能か確認する
  if(!canStart()) {
    return;
  }

  // 動作開始時点までの移動量をPositionへ反映する
  updateOdometry();

  // 派生クラス固有の準備を行う
  prepare();

  // 継続条件の初期値を設定する
  continuationCondition->prepare();

  // 継続条件を満たしている間、1周期ずつ動作を実行する
  while(continuationCondition->shouldContinue()) {
    // モーター出力など、派生クラス固有の処理を行う
    executeStep();

    // モーターが動作する時間を確保する
    wait();

    // この制御周期で移動した分をPositionへ反映する
    updateOdometry();
  }

  // モーター停止など、動作終了後の処理を行う
  finish();

  // 継続条件の判定から停止までに移動した分を反映する
  updateOdometry();
}

bool BaseMotion::canStart()
{
  return true;
}

void BaseMotion::prepare() {}

void BaseMotion::wait()
{
  ClockUtil::sleep();  // 10ミリ秒待機
}

void BaseMotion::finish() {}

void BaseMotion::updateOdometry()
{
  WheelMotorController& wheelMotorController = robot.getWheelMotorControllerInstance();

  const int32_t leftCount = wheelMotorController.getLeftCount();

  const int32_t rightCount = wheelMotorController.getRightCount();

  const double heading = robot.getIMUControllerInstance().getAzimuth();

  robot.getOdometryInstance().update(leftCount, rightCount, heading);
}