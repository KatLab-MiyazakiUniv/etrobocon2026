/**
 * @file   ETZumoApproach.h
 * @brief  ボトルデリバリー後、QR補正と外周経由でET相撲開始位置まで移動する
 */
#ifndef ET_ZUMO_APPROACH_H
#define ET_ZUMO_APPROACH_H

#include <memory>
#include "BaseMotion.h"
#include "Pid.h"

class ETZumoApproach : public BaseMotion {
 public:
  ETZumoApproach(Robot& robot,
                 std::unique_ptr<BaseContinuationCondition> condition,
                 const Pid::PidGain& rotationPid,
                 const Pid::PidGain& adjustmentPid,
                 const Pid::PidGain& straightPid);

  // AreaMasterで失敗を検知して後続動作を止める際に利用できる。
  bool wasSuccessful() const { return successful; }

 protected:
  void prepare() override;
  void executeStep() override;
  void finish() override;

 private:
  bool completed = false;
  bool successful = false;
  Pid::PidGain rotationPid;
  Pid::PidGain adjustmentPid;
  Pid::PidGain straightPid;
};

#endif  // ET_ZUMO_APPROACH_H
