/**
 * @file   RelativeRotation.h
 * @brief  相対角度で回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef RELATIVE_ROTATION_H
#define RELATIVE_ROTATION_H

#include "Rotation.h"

class RelativeRotation : public Rotation {
 public:
  RelativeRotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   const Pid::PidGain& _anglePidGain, double _deltaAngle);

 protected:
  // 現在角度にdeltaAngleを加算し、目標角度を算出する
  void prepare() override;

 private:
  // 回頭したい相対角度
  double deltaAngle;
};

#endif