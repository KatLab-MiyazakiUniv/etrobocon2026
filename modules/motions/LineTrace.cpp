/**
 * @file   LineTrace.cpp
 * @brief  ライントレース動作を実行するクラス
 * @author okuyama0528
 */

#include "LineTrace.h"

LineTrace::LineTrace(Robot& _robot,
                     std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                     double _targetSpeed, int _targetBrightness, const Pid::PidGain& _pidGain,
                     const Pid::PidGain& _rightPidGain, const Pid::PidGain& _leftPidGain)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetBrightness(_targetBrightness),
    pidGain(_pidGain),  // 誤差制御のため目標値0扱いに
    speedCalculator(_robot, _rightPidGain, _leftPidGain, _targetSpeed)
{
}

void LineTrace::prepare()
{
  // Robotから取得したコース設定に基づきエッジを決定
  edgeSign = (robot.getEdge() == Edge::LeftEdge) ? -1 : 1;
}

void LineTrace::executeStep()
{
  Pid pid(pidGain.kp, pidGain.ki, pidGain.kd, targetBrightness);
  // 直進用のベースパワーを計算
  double baseRightPower = speedCalculator.calculateRightMotorPower();
  double baseLeftPower = speedCalculator.calculateLeftMotorPower();

  // 3. PID制御による旋回パワーの計算（誤差を入力）
  // 偏差が0になれば、turningPowerは0に近づく
  double turningPower
      = pid.calculatePid(robot.getColorSensorControllerInstance().getReflectance()) * edgeSign;

  double rightPower = baseRightPower > 0.0 ? std::max(baseRightPower - turningPower, 0.0)
                                           : std::min(baseRightPower + turningPower, 0.0);
  double leftPower = baseLeftPower > 0.0 ? std::max(baseLeftPower + turningPower, 0.0)
                                         : std::min(baseLeftPower - turningPower, 0.0);
  // 4. モータ出力の決定
  // 右旋回が必要な時は右を弱め左を強める構成
  robot.getWheelMotorControllerInstance().setRightPower(rightPower);
  robot.getWheelMotorControllerInstance().setLeftPower(leftPower);
}

void LineTrace::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}