/**
 * @file   LineTrace.h
 * @brief  ライントレース動作を実行するクラス
 * @author okuyama0528
 */

#ifndef LINE_TRACE_H
#define LINE_TRACE_H

#include "BaseMotion.h"
#include "Pid.h"
#include "SpeedCalculator.h"
#include <memory>

class LineTrace : public BaseMotion {
 public:
  /**
   * @brief コンストラクタ
   * @param _rightPid 右タイヤ用走行速度制御PIDゲイン
   * @param _leftPid 左タイヤ用走行速度制御PIDゲイン
   * @param _tracePidGain ライントレース用PIDゲイン
   */
  LineTrace(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
            double _targetSpeed, int _targetBrightness, const Pid::PidGain& _pidGain,
            const Pid::PidGain& _rightPidGain, const Pid::PidGain& _leftPidGain);

 protected:
  void prepare() override;
  void executeStep() override;
  void finish() override;

 private:
  double targetSpeed;
  int targetBrightness;
  int edgeSign;  // エッジの左右判定に基づく符号（左エッジ: -1, 右エッジ: 1）
  Pid::PidGain pidGain;
  SpeedCalculator speedCalculator;
};

#endif