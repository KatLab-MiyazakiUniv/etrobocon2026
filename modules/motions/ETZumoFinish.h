#ifndef ET_ZUMO_FINISH_H
#define ET_ZUMO_FINISH_H

#include "BaseMotion.h"
#include "ProjectedMileage.h"
#include "Mileage.h"
#include <vector>

// ET相撲からフィニッシュまでを一度だけ順に実行する。
class ETZumoFinish : public BaseMotion {
 public:
  ETZumoFinish(Robot& robot, std::unique_ptr<BaseContinuationCondition> condition,
               std::vector<std::unique_ptr<BaseMotion>> children,
               std::shared_ptr<ProjectedMileage> sharedMileage
               = std::make_shared<ProjectedMileage>())
    : BaseMotion(robot, std::move(condition)),
      motions(std::move(children)),
      mileage(std::move(sharedMileage))
  {
  }

 protected:
  void prepare() override
  {
    completed = false;
    robot.getWheelMotorControllerInstance().stopBoth();
    robot.getIMUControllerInstance().resetAzimuth();
    auto& wheels = robot.getWheelMotorControllerInstance();
    mileage->reset(Mileage::calculateMileage(wheels.getRightCount(), wheels.getLeftCount()), 0.0);
  }
  void executeStep() override
  {
    if(completed) return;
    for(auto& motion : motions) {
      if(!mileage->isValid()) break;
      motion->run();
      robot.getWheelMotorControllerInstance().stopBoth();
    }
    completed = true;
  }
  void finish() override
  {
    robot.getWheelMotorControllerInstance().stopBoth();
    Logger::printfLog(Logger::INFO, "ETZumoFinish: X=%.1f mm, Y=%.1f mm",
                      mileage->getHorizontalDistance(), mileage->getVerticalDistance());
  }

 private:
  std::vector<std::unique_ptr<BaseMotion>> motions;
  std::shared_ptr<ProjectedMileage> mileage;
  bool completed = false;
};
#endif
