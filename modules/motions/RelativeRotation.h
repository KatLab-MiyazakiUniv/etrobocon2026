/**
 * @file   RelativeRotation.h
 * @brief  相対角度で回頭動作を実行するクラス
 * @author okuyama0528
 */
#ifndef RELATIVE_ROTATION_H
#define RELATIVE_ROTATION_H

#include "Rotation.h"

class RelativeRotation : public Rotation {
 public:
  RelativeRotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   double _deltaAngle);

  void prepare() override;

 private:
  double deltaAngle;
};

#endif