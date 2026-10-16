/**
 * @file   EtRallyMotion.h
 * @brief  ETラリー全体の動作を管理するクラス
 */
#ifndef ET_RALLY_MOTION_H
#define ET_RALLY_MOTION_H

#include <memory>
#include "BaseMotion.h"
#include "Pid.h"
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include "AngleNormalizer.h"
#include "ClockUtil.h"
#include "Course.h"
#include "CourseMirror.h"
#include "DijkstraRoutePlanner.h"
#include "DistanceCondition.h"
#include "EtRallyMap.h"
#include "GateRoutePlanner.h"
#include "Logger.h"
#include "MapData.h"
#include "Pid.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "RouteFollower.h"
#include "RouteTypes.h"
#include "SquareAngleAdjustment.h"
#include "Straight.h"
#include "SystemInfo.h"

class EtRallyMotion : public BaseMotion {
 public:
  EtRallyMotion(Robot& _robot,
                std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                const Pid::PidGain& _rotationPid,
                const Pid::PidGain& _squareRotationPid,
                const Pid::PidGain& _straightAnglePid);

 protected:
  void prepare() override;
  void executeStep() override;
  void finish() override;

 private:
  bool completed = false;
  Pid::PidGain rotationPid;        // 90度回頭などの通常回頭PID
  Pid::PidGain squareRotationPid;  // QR補正などの調整回頭PID
  Pid::PidGain straightAnglePid;   // 直進角度PID
};

#endif  // ET_RALLY_MOTION_H
