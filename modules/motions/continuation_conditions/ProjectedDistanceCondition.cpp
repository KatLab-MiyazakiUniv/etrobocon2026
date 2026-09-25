/**
 * @file   ProjectedDistanceCondition.cpp
 * @brief  共通座標を更新し、子条件または目標座標で動作の継続を判定するクラス
 * @author miyahara046
 */

#include "ProjectedDistanceCondition.h"
#include "Logger.h"
#include "Mileage.h"
#include <cmath>
#include <utility>

ProjectedDistanceCondition::ProjectedDistanceCondition(
    Robot& robot, std::shared_ptr<ProjectedMileage> mileage,
    std::unique_ptr<BaseContinuationCondition> child)
  : BaseContinuationCondition(robot), mileage(std::move(mileage)), child(std::move(child))
{
  LOG_CREATE("ProjectedDistanceCondition");
}

ProjectedDistanceCondition::ProjectedDistanceCondition(Robot& robot,
                                                       std::shared_ptr<ProjectedMileage> mileage,
                                                       Axis axis, double target)
  : BaseContinuationCondition(robot),
    mileage(std::move(mileage)),
    axis(axis),
    target(target),
    useTarget(true)
{
  LOG_CREATE("ProjectedDistanceCondition");
}

ProjectedDistanceCondition::~ProjectedDistanceCondition()
{
  LOG_DESTROY("ProjectedDistanceCondition");
}

void ProjectedDistanceCondition::prepare()
{
  if(child) child->prepare();
}

bool ProjectedDistanceCondition::shouldContinue()
{
  if(!mileage || !mileage->isValid() || (useTarget && !std::isfinite(target))) return false;
  auto& wheels = robot.getWheelMotorControllerInstance();
  mileage->update(Mileage::calculateMileage(wheels.getRightCount(), wheels.getLeftCount()),
                  robot.getIMUControllerInstance().getAzimuth());
  if(!mileage->isValid()) return false;
  if(!useTarget) return child && child->shouldContinue();
  double position = axis == Axis::HORIZONTAL ? mileage->getHorizontalDistance()
                                             : mileage->getVerticalDistance();
  // 正の目標は以上、負の目標は以下になったら終了する
  return target >= 0.0 ? position < target : position > target;
}
