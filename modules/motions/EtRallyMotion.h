/**
 * @file   EtRallyMotion.h
 * @brief  ETラリー全体の動作を管理するクラス
 * @author okuyama0528
 */

#ifndef ET_RALLY_MOTION_H
#define ET_RALLY_MOTION_H

#include "EtRallyCompletionCondition.h"
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

/**
 * @brief ETラリー全体の動作を管理するクラス
 */
class EtRallyMotion : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @brief EtRallyMotionを初期化する
   * @param _robot ロボットクラスのインスタンス
   * @param _continuationCondition 動作を継続する条件を判定するクラスのインスタンス
   */
  EtRallyMotion(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition);

 protected:
  /**
   * @brief ETラリー開始前に必要な準備を行う
   */
  void prepare() override;

  /**
   * @brief ETラリーの走行処理を実行する
   */
  void executeStep() override;

  /**
   * @brief ETラリー終了後の処理を行う
   */
  void finish() override;

 private:
  bool completed = false;  // ETラリー全体の動作が完了したかどうか
};

#endif  // ET_RALLY_MOTION_H