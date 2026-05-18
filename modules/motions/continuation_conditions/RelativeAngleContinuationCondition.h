#ifndef RELATIVE_ANGLE_CONTINUATION_CONDITION_H
#define RELATIVE_ANGLE_CONTINUATION_CONDITION_H

#include "AngleContinuationCondition.h"

class RelativeAngleContinuationCondition : public AngleContinuationCondition {
 public:
  RelativeAngleContinuationCondition(Robot& robot, double _deltaAngle, double _tolerance = 2.0);

  void prepare() override;

 private:
  double deltaAngle;
};

#endif
