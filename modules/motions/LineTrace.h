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
#include "Logger.h"
#include "CsvLogger.h"
#include <string>

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
   * @param _deadbandRate turningPowerに適用するデッドバンドの割合(0.0〜1.0、basePowerに対する割合)
   * @param _maxoutRate turningPowerに適用するマックスアウトの割合(0.0〜1.0、basePowerに対する割合)
   */
  LineTrace(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
            double _targetSpeed, int _targetBrightness, const Pid::PidGain& _brightnessPidGain,
            double _deadbandRate, double _maxoutRate, const std::string& _commandId = "");

  /**
   * デストラクタ
   */
  ~LineTrace();

 protected:
  /**
   * @brief 動作を開始できるかを判定する
   * @return true/動作を開始できる、false/動作を開始できない
   */
  bool canStart() override;

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
  double targetSpeed;    // 目標速度(mm/秒)
  int targetBrightness;  // 目標とする明るさの値(%)
  int edgeSign;  // エッジの左右判定に基づく符号（左エッジ: -1, 右エッジ: 1）
  Pid brightnessPid;  // ライントレース用の輝度値に対するPID
  SpeedCalculator speedCalculator;  // 目標速度に対する左右車輪のPID制御を行うクラスのインスタンス
  double deadbandRate;  // turningPowerに適用するデッドバンドの割合(0.0〜1.0)
  double maxoutRate;    // turningPowerに適用するマックスアウトの割合(0.0〜1.0)
  Pid::PidGain brightnessPidGain;  // ログ出力用のPIDゲイン
  std::string commandId;           // ログ上で区間を識別するコマンドID
};

#endif
