#ifndef PROJECTED_DISTANCE_CONDITION_H
#define PROJECTED_DISTANCE_CONDITION_H

#include "BaseContinuationCondition.h"
#include "ProjectedMileage.h"
#include "Mileage.h"
#include <memory>
#include <cmath>

// 共通座標を毎周期更新する。子条件のみ、または軸の目標座標で終了する。
class ProjectedDistanceCondition : public BaseContinuationCondition {
 public:
  enum class Axis { HORIZONTAL, VERTICAL };
  ProjectedDistanceCondition(Robot& robot, std::shared_ptr<ProjectedMileage> mileage,
                             std::unique_ptr<BaseContinuationCondition> child)
    : BaseContinuationCondition(robot), mileage(std::move(mileage)), child(std::move(child))
  {
  }
  ProjectedDistanceCondition(Robot& robot, std::shared_ptr<ProjectedMileage> mileage, Axis axis,
                             double target)
    : BaseContinuationCondition(robot),
      mileage(std::move(mileage)),
      axis(axis),
      target(target),
      useTarget(true)
  {
  }
  void prepare() override
  {
    if(child) child->prepare();
  }
  bool shouldContinue() override
  {
    if(!mileage || !mileage->isValid() || (useTarget && !std::isfinite(target))) return false;
    auto& wheels = robot.getWheelMotorControllerInstance();
    mileage->update(Mileage::calculateMileage(wheels.getRightCount(), wheels.getLeftCount()),
                    robot.getIMUControllerInstance().getAzimuth());
    if(!mileage->isValid()) return false;
    if(!useTarget) return child && child->shouldContinue();
    double position = axis == Axis::HORIZONTAL ? mileage->getHorizontalDistance()
                                               : mileage->getVerticalDistance();
    // 正の目標は以上、負の目標は以下になったら終了する。
    return target >= 0.0 ? position < target : position > target;
  }

 private:
  std::shared_ptr<ProjectedMileage> mileage;
  std::unique_ptr<BaseContinuationCondition> child;
  Axis axis = Axis::HORIZONTAL;
  double target = 0.0;
  bool useTarget = false;
};
#endif
