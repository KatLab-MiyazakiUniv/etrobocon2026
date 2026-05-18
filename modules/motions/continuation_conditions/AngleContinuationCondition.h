#ifndef ANGLE_CONTINUATION_CONDITION_H
#define ANGLE_CONTINUATION_CONDITION_H

#include "BaseContinuationCondition.h"

class AngleContinuationCondition : public BaseContinuationCondition {
 public:
  AngleContinuationCondition(Robot& robot, double _tolerance = 2.0);

  bool shouldContinue() override;

 protected:
  double targetAngle;
  double normalizeAngle(double angle);

 private:
  double tolerance;
};

#endif
