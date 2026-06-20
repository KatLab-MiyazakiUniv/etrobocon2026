/**
 * @file   LineTrace.h
 * @brief  ライントレース動作を実行するクラス
 * @author okuyama0528
 */

#ifndef LINE_TRACE_H
#define LINE_TRACE_H

#include "BaseMotion.h"
#include "Pid.h"
#include "SpeedCalculator.h"
#include <memory>
#include "Logger.h"

class LineTrace : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @brief LineTrace を初期化する
   * @param _robot ロボットクラスのインスタンス
   * @param _continuationCondition 動作を継続する条件を判定するクラスのインスタンス
   * @param _targetSpeed 目標速度(mm/秒)
   * @param _targetBrightness 目標とする明るさの値(%)
   * @param _brightnessPidGain ライントレース用の輝度値に対するPIDゲイン
   */
  LineTrace(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
            double _targetSpeed, int _targetBrightness, const Pid::PidGain& _brightnessPidGain);

  /**
   * デストラクタ
   */
  ~LineTrace();

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
   * @brief 両タイヤモータを停止する
   */
  void finish() override;

 private:
  double targetSpeed;               // 目標速度(mm/秒)
  int targetBrightness;             // 目標とする明るさの値(%)
  int edgeSign;                     // エッジの左右判定に基づく符号（左エッジ: -1, 右エッジ: 1）
  Pid::PidGain brightnessPidGain;   // ライントレース用の輝度値に対するPIDゲイン
  SpeedCalculator speedCalculator;  // 目標速度に対する左右車輪のPID制御を行うクラスのインスタンス
};

#endif