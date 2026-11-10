/**
 * @file   EtRallyFinish.h
 * @brief  ETラリー後のゴール走行を行うクラス
 * @author okuyama0528
 */

#ifndef ET_RALLY_FINISH_H
#define ET_RALLY_FINISH_H

#include <memory>
#include "BaseMotion.h"
#include "Straight.h"
#include "RelativeRotation.h"
#include "DistanceCondition.h"
#include "RelativeAngleCondition.h"

class EtRallyFinish : public BaseMotion {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot ロボット
   * @param _continuationCondition 継続条件
   * @param _rotationPid 回頭PID
   * @param _straightAnglePid 直進角度PID
   * @param _targetSpeed 走行速度
   * @param _rotationAngle 回頭角度[deg]
   * @param _straightDistance 直進距離[mm]
   * @param _straightDeadbandRate 直進デッドバンド率
   * @param _straightMaxoutRate 直進マックスアウト率
   */
  explicit EtRallyFinish(Robot& _robot,
                         std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                         const Pid::PidGain& _rotationPid, const Pid::PidGain& _straightAnglePid,
                         double _rotationAngle, double _straightDistance, double _targetSpeed,
                         double _straightDeadbandRate, double _straightMaxoutRate);

 protected:
  /**
   * @brief 走行前の準備処理
   */
  void prepare() override;

  /**
   * @brief ゴールまでの走行処理
   */
  void executeStep() override;

  /**
   * @brief 走行終了処理
   */
  void finish() override;

 private:
  Pid::PidGain rotationPid;       // 回頭PIDゲイン
  Pid::PidGain straightAnglePid;  // 直進角度PIDゲイン
  double targetSpeed;             // 目標走行速度
  double rotationAngle;           // 回頭角度[deg]
  double straightDistance;        // 直進距離[mm]
  double straightDeadbandRate;    // 直進デッドバンド率
  double straightMaxoutRate;      // 直進マックスアウト率
};

#endif  // ET_RALLY_FINISH_H