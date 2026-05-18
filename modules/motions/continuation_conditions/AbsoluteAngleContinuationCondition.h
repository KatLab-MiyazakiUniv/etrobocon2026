#ifndef ABSOLUTE_ANGLE_CONTINUATION_CONDITION_H
#define ABSOLUTE_ANGLE_CONTINUATION_CONDITION_H

#include "AngleContinuationCondition.h"

class AbsoluteAngleContinuationCondition : public AngleContinuationCondition {
 public:
  AbsoluteAngleContinuationCondition(Robot& robot, double _specifiedTargetAngle,
                                     double _tolerance = 2.0);

  void prepare() override;

 private:
  double specifiedTargetAngle;
};

#endif
