/**
 * @file ETZumoExit.cpp
 * @brief 基準方向の距離を使ってETZumoエリアを突破する複合動作
 * @author miyahara046
 */

#include "ETZumoExit.h"

ETZumoExit::ETZumoExit(Robot& _robot,
                       std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                       std::shared_ptr<ProjectedMileage> _mileage,
                       std::vector<std::unique_ptr<BaseMotion>> _motions)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    mileage(std::move(_mileage)),
    motions(std::move(_motions))
{
  LOG_CREATE("ETZumoExit");
}

ETZumoExit::~ETZumoExit()
{
  LOG_DESTROY("ETZumoExit");
}

bool ETZumoExit::canStart()
{
  if(!mileage) {
    Logger::error("ETZumoExit: 距離計測のインスタンスが設定されていません");
    robot.getWheelMotorControllerInstance().stopBoth();
    return false;
  }
  if(motions.size() != 3) {
    Logger::error("ETZumoExit: 追尾・回頭・直進の3つの子動作を設定してください");
    robot.getWheelMotorControllerInstance().stopBoth();
    return false;
  }
  for(size_t i = 0; i < motions.size(); ++i) {
    if(!motions[i]) {
      Logger::printfLog(Logger::ERROR, "ETZumoExit: 子動作[%zu]が設定されていません", i);
      robot.getWheelMotorControllerInstance().stopBoth();
      return false;
    }
  }
  return true;
}

void ETZumoExit::prepare()
{
  robot.getWheelMotorControllerInstance().stopBoth();
  // この向きを0度とし、追尾開始前の車輪位置を距離計測の起点にする。
  robot.getIMUControllerInstance().resetAzimuth();
  mileage->reset(Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                           robot.getWheelMotorControllerInstance().getLeftCount()),
                 0.0);
}

void ETZumoExit::executeStep()
{
  // リストは追尾→回頭→直進の順。計測異常時は後続の子動作を実行しない。
  for(auto& motion : motions) {
    if(!mileage->isValid()) {
      Logger::error("ETZumoExit: 距離計測の異常により残りの子動作を中止します");
      robot.getWheelMotorControllerInstance().stopBoth();
      return;
    }
    // 各子動作の継続条件が共通の距離を更新し、目標到達を判定する。
    motion->run();
    robot.getWheelMotorControllerInstance().stopBoth();
  }
}

void ETZumoExit::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
  Logger::printfLog(Logger::INFO, "ETZumoExit: X=%.1f mm, Y=%.1f mm",
                    mileage->getHorizontalDistance(), mileage->getVerticalDistance());
}
