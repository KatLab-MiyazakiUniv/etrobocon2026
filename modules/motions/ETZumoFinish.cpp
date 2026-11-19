/**
 * @file ETZumoFinish.cpp
 * @brief ET相撲からフィニッシュまでの子動作を順に実行する複合動作
 */

#include "ETZumoFinish.h"
#include "Mileage.h"
#include "Logger.h"
#include "MapData.h"
#include <utility>

namespace {

  /**
   * @brief ETラリーに必要な3色のゲート情報がそろっているか判定する
   */
  bool hasAllGates(const MapData& mapData)
  {
    bool hasRed = false;
    bool hasBlue = false;
    bool hasYellow = false;

    for(const auto& gate : mapData.getGates()) {
      if(gate.color == GoalColor::RED) {
        hasRed = true;
      } else if(gate.color == GoalColor::BLUE) {
        hasBlue = true;
      } else if(gate.color == GoalColor::YELLOW) {
        hasYellow = true;
      }
    }

    return hasRed && hasBlue && hasYellow;
  }

}  // namespace

ETZumoFinish::ETZumoFinish(Robot& robot, std::unique_ptr<BaseContinuationCondition> condition,
                           std::vector<std::unique_ptr<BaseMotion>> children,
                           std::shared_ptr<ProjectedMileage> sharedMileage)
  : BaseMotion(robot, std::move(condition)),
    motions(std::move(children)),
    mileage(std::move(sharedMileage))
{
}

void ETZumoFinish::prepare()
{
  completed = false;
  robot.getWheelMotorControllerInstance().stopBoth();
  robot.getIMUControllerInstance().resetAzimuth();
  mileage->reset(Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                           robot.getWheelMotorControllerInstance().getLeftCount()),
                 0.0);
}

void ETZumoFinish::executeStep()
{
  if(completed) return;

  for(size_t i = 0; i < motions.size(); ++i) {
    if(!mileage->isValid()) break;

    motions[i]->run();
    robot.getWheelMotorControllerInstance().stopBoth();

    // ETZumoFinish.csvの先頭の子動作はETZumoExit。
    // 退出後にゲート情報がそろっていれば、地区大会用ゴール経路を実行しない。
    if(i == 0) {
      if(!mileage->isValid()) {
        Logger::error("ETZumoFinish: ETZumoExit後に距離計測が無効になりました");
        break;
      }

      if(hasAllGates(robot.getMapDataInstance())) {
        Logger::info("ETZumoFinish: ゲート情報が3色そろったためETラリーへ移行します");
        break;
      }

      Logger::info("ETZumoFinish: ゲート情報不足のため地区大会用ゴール経路を実行します");
    }
  }
  completed = true;
}

void ETZumoFinish::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
  Logger::printfLog(Logger::INFO, "ETZumoFinish: X=%.1f mm, Y=%.1f mm",
                    mileage->getHorizontalDistance(), mileage->getVerticalDistance());
}