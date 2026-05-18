#ifndef ANGLE_CONTINUATION_CONDITION_H
#define ANGLE_CONTINUATION_CONDITION_H

#include "BaseContinuationCondition.h"

class AngleContinuationCondition : public BaseContinuationCondition {
 public:
  AngleContinuationCondition(Robot& robot, double _targetAngle, double _tolerance = 2.0);

  void prepare() override;
  bool shouldContinue() override;

 private:
  double targetAngle;
  double tolerance;

  double initialAngle;
  double currentAngle;

  double normalizeAngle(double angle);
};

#endif