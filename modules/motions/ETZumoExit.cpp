/**
 * @file ETZumoExit.cpp
 * @brief 基準方向の距離を使ってETZumoエリアを突破する複合動作
 * @author miyahara046
 */
/**
 * @file ETZumoExit.cpp
 * @brief 追尾・回頭・直進を順番に実行する複合動作
 */

#include "ETZumoExit.h"

ETZumoExit::ETZumoExit(Robot& _robot,
                       std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                       std::vector<std::unique_ptr<BaseMotion>> _motions)
  : BaseMotion(_robot, std::move(_continuationCondition)), motions(std::move(_motions))
{
  LOG_CREATE("ETZumoExit");
}

ETZumoExit::~ETZumoExit()
{
  LOG_DESTROY("ETZumoExit");
}

bool ETZumoExit::canStart()
{
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

void ETZumoExit::executeStep()
{
  // 追尾 → 回頭 → 直進の順に実行する。
  for(auto& motion : motions) {
    motion->run();
    robot.getWheelMotorControllerInstance().stopBoth();
  }
}

void ETZumoExit::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
  Logger::printfLog(Logger::INFO, "ETZumoExit: 動作終了");
}