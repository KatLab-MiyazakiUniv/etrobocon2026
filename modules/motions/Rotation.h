/**
 * @file   Rotation.h
 * @brief  回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef ROTATION_H
#define ROTATION_H

#include "BaseMotion.h"
#include "Pid.h"

class Rotation : public BaseMotion {
 public:
  Rotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition);

  void prepare() override;

 protected:
  void executeStep() override;
  void finish() override;

  double normalizeAngle(double angle);

 protected:
  Pid pid;
  double currentRightPower;
  double currentLeftPower;
  double initialAngle;
  double targetAngle;
  double getCurrentAngle();
};

#endif