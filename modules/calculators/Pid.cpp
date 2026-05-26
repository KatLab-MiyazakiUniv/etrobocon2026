/**
 * @file Pid.cpp
 * @brief PIDを計算するクラス
 * @author migaku2645
 */

#include "Pid.h"
#include <iostream>

Pid::Pid(double _kp, double _ki, double _kd, double _targetValue, double _maxIntegral,
         double _minIntegral)
  : pidGain(_kp, _ki, _kd),
    targetValue(_targetValue),
    maxIntegral(_maxIntegral),
    minIntegral(_minIntegral)
{
}

Pid::Pid(double _kp, double _ki, double _kd, double _targetValue)
  : Pid(_kp, _ki, _kd, _targetValue, 100.0, -100.0)
{
}

Pid::PidGain::PidGain(double _kp, double _ki, double _kd)
{
  // Pidゲインのいずれかが負の場合、一括で警告を出す
  if(_kp < 0 || _ki < 0 || _kd < 0) {
    Logger::warning("PID gains cannot be negative. Negative values are clamped to 0.0.");
  }

  // 負の値は 0.0 に補正
  kp = (_kp < 0) ? 0.0 : _kp;
  ki = (_ki < 0) ? 0.0 : _ki;
  kd = (_kd < 0) ? 0.0 : _kd;
}

void Pid::setPidGain(double _kp, double _ki, double _kd)
{
  // Pidゲインのいずれかが負の場合、一括で警告を出す
  if(_kp < 0 || _ki < 0 || _kd < 0) {
    Logger::warning("Attempted to set negative PID gains. Clamped to 0.0.");
  }
  // 負の値は 0.0 に補正
  pidGain.kp = (_kp < 0) ? 0.0 : _kp;
  pidGain.ki = (_ki < 0) ? 0.0 : _ki;
  pidGain.kd = (_kd < 0) ? 0.0 : _kd;
}
double Pid::calculatePid(double currentValue, double delta)
{
  // 0除算を避けるために delta=0 の場合はデフォルト周期0.01とする
  if(delta == 0) delta = defaultDelta;

  // 現在の目標値との偏差を求める
  double currentDeviation = targetValue - currentValue;
  // 積分の処理を行う
  integral += (currentDeviation + prevDeviation) * delta / 2.0;
  // 累積する積分値の大きさ制限
  if(integral > maxIntegral) {
    integral = maxIntegral;
  } else if(integral < minIntegral) {
    integral = minIntegral;
  }
  // 微分の処理を行う
  double currentDerivative = (currentDeviation - prevDeviation) / delta;
  /**
   * 微分項にローパスフィルタを適用
   * 偏差が大きい際に過大な変化量を一気に与えず
   * 滑らかな変化にし、機体の暴走を防ぐため
   */
  filteredDerivative = alpha * currentDerivative + (1.0 - alpha) * filteredDerivative;

  // 前回の偏差を更新する
  prevDeviation = currentDeviation;

  // P制御の計算を行う
  double p = pidGain.kp * currentDeviation;
  // I制御の計算を行う
  double i = pidGain.ki * integral;
  // D制御の計算を行う
  double d = pidGain.kd * filteredDerivative;

  // 操作量 = P制御 + I制御 + D制御
  return (p + i + d);
}