/**
 * @file ETZumoExitCondition.cpp
 * @brief 基準方向の累積距離と子動作の継続条件で終了を判定するクラス
 * @author miyahara046
 */

#include "ETZumoExitCondition.h"
#include <cmath>

ETZumoExitCondition::ETZumoExitCondition(
    Robot& _robot, std::shared_ptr<ProjectedMileage> _mileage, double _targetDistance,
    std::unique_ptr<BaseContinuationCondition> _continuationCondition, bool _requireBoth)
  : BaseContinuationCondition(_robot),
    mileage(std::move(_mileage)),
    requireBoth(_requireBoth),
    targetDistance(_targetDistance),
    continuationCondition(std::move(_continuationCondition))
{
  LOG_CREATE("ETZumoExitCondition");
}

ETZumoExitCondition::~ETZumoExitCondition()
{
  LOG_DESTROY("ETZumoExitCondition");
}

void ETZumoExitCondition::prepare()
{
  // 子動作の条件だけを初期化する。共有する距離は動作の切り替えでリセットしない。
  // 直進では子動作の条件を省略し、基準方向の距離だけで終了を判断する。
  if(continuationCondition) {
    continuationCondition->prepare();
  }
}

bool ETZumoExitCondition::shouldContinue()
{
  // 不正な設定では継続せず、BaseMotionの終了処理へ進む。
  if(!mileage) {
    Logger::error("ETZumoExitCondition: 距離計測のインスタンスが設定されていません");
    return false;
  }
  if(!std::isfinite(targetDistance) || targetDistance <= 0.0) {
    Logger::error("ETZumoExitCondition: 目標距離は有限の正の値を指定してください");
    return false;
  }
  if(!mileage->isValid()) {
    Logger::error("ETZumoExitCondition: 距離計測が未初期化、または異常のため終了します");
    return false;
  }
  // 左右車輪の平均移動距離と現在方位から、0度方向への移動を更新する。
  mileage->update(Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                            robot.getWheelMotorControllerInstance().getLeftCount()),
                  robot.getIMUControllerInstance().getAzimuth());
  if(!mileage->isValid()) {
    Logger::error("ETZumoExitCondition: 距離の更新に失敗したため終了します");
    return false;
  }
  if(requireBoth) {
    // 距離未達の間は色を数えず、到達後の連続検知で停止する。
    return mileage->getDistance() < targetDistance
           || (continuationCondition && continuationCondition->shouldContinue());
  }
  // 共通の距離に到達したら終了する。未到達なら子動作固有の条件も確認する。
  return mileage->getDistance() < targetDistance
         && (!continuationCondition || continuationCondition->shouldContinue());
}
