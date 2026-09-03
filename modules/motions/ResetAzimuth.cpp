/**
 * @file ResetAzimuth.cpp
 * @brief 現在の機体方位を基準角度（0度）に設定する動作
 * @author miyahara046
 */
#include "ResetAzimuth.h"

ResetAzimuth::ResetAzimuth(
    Robot& robot, std::unique_ptr<BaseContinuationCondition> continuationCondition)
  : BaseMotion(robot, std::move(continuationCondition))
{
  LOG_CREATE("ResetAzimuth");
}

ResetAzimuth::~ResetAzimuth()
{
  LOG_DESTROY("ResetAzimuth");
}

void ResetAzimuth::executeStep()
{
  robot.getIMUControllerInstance().resetAzimuth();
}
