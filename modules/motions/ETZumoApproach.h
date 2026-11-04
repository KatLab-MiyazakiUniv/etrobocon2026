/**
 * @file   ETZumoApproach.h
 * @brief  ボトルデリバリー後、QR補正と外周経由でET相撲開始位置まで移動する
 * @author okuyama0528
 */
#ifndef ET_ZUMO_APPROACH_H
#define ET_ZUMO_APPROACH_H

#include <memory>
#include <cmath>
#include <functional>
#include <utility>
#include "BaseMotion.h"
#include "Pid.h"
#include "AngleNormalizer.h"
#include "Course.h"
#include "CourseMirror.h"
#include "DistanceCondition.h"
#include "EtRallyMap.h"
#include "Logger.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "RouteTypes.h"
#include "SquareAngleAdjustment.h"
#include "Straight.h"
#include "SystemInfo.h"

class ETZumoApproach : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @param robot Robotインスタンスの参照
   * @param condition 継続条件
   * @param rotationPid 回転制御用PIDゲイン
   * @param adjustmentPid QR補正用PIDゲイン
   * @param straightPid 直進制御用PIDゲイン
   */
  ETZumoApproach(Robot& robot, std::unique_ptr<BaseContinuationCondition> condition,
                 const Pid::PidGain& rotationPid, const Pid::PidGain& adjustmentPid,
                 const Pid::PidGain& straightPid, double targetSpeed, double straightDeadbandRate,
                 double straightMaxoutRate);

 protected:
  /**
   * @brief 動作を開始する前に必要な準備を行う
   */
  void prepare() override;

  /**
   * @brief 1周期分の動作を実行する
   */
  void executeStep() override;

  /**
   * @brief 動作終了後の処理を行う
   */
  void finish() override;

 private:
  Pid::PidGain rotationPid;     // 回転制御用PIDゲイン
  Pid::PidGain adjustmentPid;   // QR補正用PIDゲイン
  Pid::PidGain straightPid;     // 直進制御用PIDゲイン
  double targetSpeed;           // 目標速度
  double straightDeadbandRate;  // 直進制御のデッドバンド率
  double straightMaxoutRate;    // 直進制御の最大出力率
};

#endif  // ET_ZUMO_APPROACH_H
