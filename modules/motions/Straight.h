/**
 * @file   DistanceCondition.h
 * @brief
 * @author migaku2645
 */
#ifndef STRAIGHT_H
#define STRAIGHT_H

#include "BaseMotion.h"
#include "SpeedCalculator.h"
#include "ClockUtil.h"

class Straight : public BaseMotion {
 public:
  Straight(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
           double _targetSpeed);

  void executeStep() override;
  void wait() override;

 private:
  double targetSpeed;  // 目標速度
  SpeedCalculator speedCalculator;
};

#endif