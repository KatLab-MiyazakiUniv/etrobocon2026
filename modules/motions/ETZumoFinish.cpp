/**
 * @file ETZumoFinish.cpp
 * @brief ET相撲からフィニッシュまでの子動作を順に実行する複合動作
 */

#include "ETZumoFinish.h"
#include "Mileage.h"
#include "Logger.h"
#include <utility>

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
  for(auto& motion : motions) {
    if(!mileage->isValid()) break;
    motion->run();
    robot.getWheelMotorControllerInstance().stopBoth();
  }
  completed = true;
}

void ETZumoFinish::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
  Logger::printfLog(Logger::INFO, "ETZumoFinish: X=%.1f mm, Y=%.1f mm",
                    mileage->getHorizontalDistance(), mileage->getVerticalDistance());
}
