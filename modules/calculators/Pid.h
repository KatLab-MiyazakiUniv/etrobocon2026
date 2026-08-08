/**
 * @file Pid.h
 * @brief PIDを計算するクラス
 * @author migaku2645
 */

#ifndef PID_H
#define PID_H
#include "Logger.h"
#include "ClockUtil.h"

class Pid {
 public:
  // PIDゲインを保持する構造体
  struct PidGain {
   public:
    double kp;  // Pゲイン
    double ki;  // Iゲイン
    double kd;  // Dゲイン

    /** コンストラクタ
     * @param _kp Pゲイン
     * @param _ki Iゲイン
     * @param _kd Dゲイン
     */
    PidGain(double _kp, double _ki, double _kd);
  };

  /** コンストラクタ
   * @param _kp Pゲイン
   * @param _ki Iゲイン
   * @param _kd Dゲイン
   * @param _targetValue 目標値
   * @param _maxIntegral 累積積分値の最大値
   * @param _minIntegral 累積積分値の最小値
   */
  Pid(double _kp, double _ki, double _kd, double _targetValue, double _maxIntegral,
      double _minIntegral);

  /** 積分値制限を設定しない場合のコンストラクタ
   * @param _kp Pゲイン
   * @param _ki Iゲイン
   * @param _kd Dゲイン
   * @param _targetValue 目標値
   */
  Pid(double _kp, double _ki, double _kd, double _targetValue);

  /**
   * @brief PIDゲインを設定する
   * @param _kp Pゲイン
   * @param _ki Iゲイン
   * @param _kd Dゲイン
   */
  void setPidGain(double _kp, double _ki, double _kd);

  /**
   * @brief Pidの初回呼び出し時間を設定する
   */
  void prepare();

  /**
   * @brief PIDを計算する
   * @param currentValue 現在値
   * @return PIDの計算結果(操作量)
   */
  double calculatePid(double currentValue);

  /**
   * @brief PIDの内部状態をリセットする
   */
  void reset();

 private:
  PidGain pidGain;
  double prevTime = -1.0;               // 前回の時間
  double prevDeviation = 0.0;           // 前回の偏差
  double integral = 0.0;                // 偏差の累積
  double filteredDerivative = 0.0;      // フィルタされた微分項を保持する変数
  double targetValue;                   // 目標値
  double maxIntegral = 100.0;           // 累積積分値の最大値
  double minIntegral = -100.0;          // 累積積分値の最小値
  static constexpr double alpha = 0.8;  // ローパスフィルタの係数
};

#endif  // PID_H