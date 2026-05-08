/**
 * @file   BaseMotion.cpp
 * @brief  動作の実行処理を共通化するための基底クラス
 * @author takuchi17
 */

#include "BaseMotion.h"

BaseMotion::BaseMotion(Robot& _robot,
                       std::unique_ptr<BaseContinuationCondition> _continuationCondition)
  : robot(_robot), continuationCondition(std::move(_continuationCondition))
{
}

void BaseMotion::run()
{
  // 実行時の状態を確認する。
  // 基本はtrueで、IMUやカメラサーバーなどの状態確認が必要な動作だけcanStart関数をoverrideする。
  if(!canStart()) {
    return;
  }

  // メンバ変数の初期化など、動作固有の事前準備を行う。
  prepare();

  // 初期値のセットなど、継続条件の事前準備を行う。
  continuationCondition->prepare();

  // 継続条件を満たしている間、1周期ずつ動作を実行する。
  while(continuationCondition->shouldContinue()) {
    // 派生クラス固有の1周期分の動作を実行する。
    executeStep();

    // 制御を安定させるために待機する。
    wait();
  }

  // モーター停止など、動作終了後の処理を行う。
  finish();
}

bool BaseMotion::canStart()
{
  return true;
}

void BaseMotion::prepare() {}

void BaseMotion::wait() {}

void BaseMotion::finish() {}
