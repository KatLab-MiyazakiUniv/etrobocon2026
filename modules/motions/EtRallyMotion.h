/**
 * @file   EtRallyMotion.h
 * @brief  ETラリー全体の動作を管理するクラス
 * @author okuyama0528
 */

#ifndef ET_RALLY_MOTION_H
#define ET_RALLY_MOTION_H

#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>
#include "AngleNormalizer.h"
#include "BaseMotion.h"
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
 EtRallyMotion(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
              const Pid::PidGain& _rotationPid, const Pid::PidGain& _squareRotationPid,
              const Pid::PidGain& _straightAnglePid, double _targetSpeed,
              double _straightDeadbandRate, double _straightMaxoutRate);
 protected:
 /**
   * @brief 走行前の準備処理
   */
  void prepare() override;

  /**
   * @brief 走行処理
   */
  void executeStep() override;

  /**
   * @brief 走行後の終了処理
   */
  void finish() override;

 private:
  bool completed = false;
  Pid::PidGain rotationPid;        // 90度回頭などの通常回頭PID
  Pid::PidGain squareRotationPid;  // QR補正などの調整回頭PID
  Pid::PidGain straightAnglePid;   // 直進角度PID
    double targetSpeed;            // 走行速度
  double straightDeadbandRate;   // 直進時のデッドバンド率
  double straightMaxoutRate;     // 直進時の最大出力率
};

#endif  // ET_RALLY_MOTION_H
