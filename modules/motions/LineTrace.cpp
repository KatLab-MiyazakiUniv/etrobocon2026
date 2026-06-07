/**
 * @file   LineTrace.cpp
 * @brief  ライントレース動作を実行するクラス
 * @author okuyama0528
 */

#include "LineTrace.h"

LineTrace::LineTrace(Robot& _robot,
                     std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                     double _targetSpeed, int _targetBrightness, const Pid::PidGain& _rightPid,
                     const Pid::PidGain& _leftPid, const Pid::PidGain& _tracePidGain)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetBrightness(_targetBrightness),
    pid(_tracePidGain.kp, _tracePidGain.ki, _tracePidGain.kd, 0.0),  // 誤差制御のため目標値0扱いに
    speedCalculator(_robot, _rightPid, _leftPid, _targetSpeed)
{
}

void LineTrace::prepare()
{
  // Robotから取得したコース設定に基づきエッジを決定
  edgeSign = (robot.getCourse() == Course::Left) ? -1 : 1;
}

void LineTrace::executeStep()
{
  // 1. 直進用のベースパワーを計算
  double baseRightPower = speedCalculator.calculateRightMotorPower();
  double baseLeftPower = speedCalculator.calculateLeftMotorPower();

  // 2. ライントレース制御誤差（偏差）の計算
  // 偏差 = 目標輝度 - 現在輝度
  double currentBrightness
      = static_cast<double>(robot.getColorSensorControllerInstance().getReflectance());
  double error = static_cast<double>(targetBrightness) - currentBrightness;

  // 3. PID制御による旋回パワーの計算（誤差を入力）
  // 偏差が0になれば、turningPowerは0に近づく
  double turningPower = pid.calculatePid(error) * edgeSign;

  // 4. モータ出力の決定
  // 右旋回が必要な時は右を弱め左を強める構成
  robot.getWheelMotorControllerInstance().setRightPower(baseRightPower - turningPower);
  robot.getWheelMotorControllerInstance().setLeftPower(baseLeftPower + turningPower);
}

void LineTrace::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}