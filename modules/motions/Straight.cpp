/**
 * @file   Straight.cpp
 * @brief  直進動作を実行するクラス
 * @author migaku2645
 */

#include "Straight.h"

Straight::Straight(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   double _targetSpeed, const Pid::PidGain& _rightPid, const Pid::PidGain& _leftPid,
                   const Pid::PidGain& _anglePidGain, bool _shouldUseIMU)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    speedCalculator(_robot, _rightPid, _leftPid, _targetSpeed),
    anglePid(_anglePidGain.kp, _anglePidGain.ki, _anglePidGain.kd, 0.0),
    shouldUseIMU(_shouldUseIMU),
    targetAngle(0.0)
{
}

bool Straight::canStart()
{
  // targetSpeed値が0の場合は終了する
  if(targetSpeed == 0.0) {
    return false;
  }
  return true;
}

void Straight::prepare()
{
  // IMU有りの場合、走行前の角度を目標角度に設定
  if(shouldUseIMU) {
    targetAngle = robot.getIMUControllerInstance().getAzimuth();
  }
}

void Straight::executeStep()
{
  double requiredRightPower = speedCalculator.calculateRightMotorPower();
  double requiredLeftPower = speedCalculator.calculateLeftMotorPower();
  double turningPower = 0.0;

  if(shouldUseIMU) {
    // 角度のズレを補正する
    double currentAngle = robot.getIMUControllerInstance().getAzimuth();
    double angleDeviation = targetAngle - currentAngle;

    // 角度の誤差を-180度から180度の範囲に正規化
    if(angleDeviation > 180.0) {
      angleDeviation -= 360.0;
    } else if(angleDeviation < -180.0) {
      angleDeviation += 360.0;
    }

    turningPower = anglePid.calculatePid(angleDeviation);
  }

  // モーターにPower値をセット
  robot.getWheelMotorControllerInstance().setRightPower(requiredRightPower + turningPower);
  robot.getWheelMotorControllerInstance().setLeftPower(requiredLeftPower - turningPower);
}

void Straight::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}
