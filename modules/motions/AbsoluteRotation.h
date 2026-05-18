/**
 * @file   AbsoluteRotation.h
 * @brief  絶対角度で回頭動作を実行するクラス
 * @author okuyama0528
 */
#ifndef ABSOLUTE_ROTATION_H
#define ABSOLUTE_ROTATION_H

#include "Rotation.h"

class AbsoluteRotation : public Rotation {
 public:
  AbsoluteRotation(Robot& _robot,
                   std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   const Pid::PidGain& _anglePidGain, double _specifiedTargetAngle);

 protected:
  void prepare() override;

 private:
  double specifiedTargetAngle;
};

#endif
