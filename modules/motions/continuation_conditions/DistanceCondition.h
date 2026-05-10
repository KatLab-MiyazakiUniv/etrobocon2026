/**
 * @file   DistanceCondition.h
 * @brief
 * @author migaku2645
 */
#ifndef DISTANCE_CONDITION_H
#define DISTANCE_CONDITION_H

#include "BaseContinuationCondition.h"
#include "Mileage.h"
#include <cmath>

class DistanceCondition : public BaseContinuationCondition {
 public:
  DistanceCondition(Robot& _robot, double _targetDistance);

  void prepare() override;
  bool shouldContinue() override;

 private:
  double targetDistance;   // 目標距離
  double initDistance;     // 開始時の累計走行距離
  double currentDistance;  // 開始時からの移動距離
};

#endif