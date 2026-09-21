/**
 * @file   Straight.cpp
 * @brief  直進動作を実行するクラス
 * @author migaku2645
 */

#include "Straight.h"

Straight::Straight(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   double _targetSpeed, const Pid::PidGain& _rightPid, const Pid::PidGain& _leftPid,
                   const Pid::PidGain& _anglePidGain, bool _shouldUseIMU, double _deadbandRate,
                   double _maxoutRate)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    speedCalculator(_robot, _rightPid, _leftPid, _targetSpeed),
    anglePid(_anglePidGain.kp, _anglePidGain.ki, _anglePidGain.kd, 0.0),
    shouldUseIMU(_shouldUseIMU),
    targetAngle(0.0),
    deadbandRate(_deadbandRate),
    maxoutRate(_maxoutRate)
{
  LOG_CREATE("Straight");
}

Straight::~Straight()
{
  LOG_DESTROY("Straight");
}

bool Straight::canStart()
{
  // targetSpeed値が0の場合は終了する
  if(targetSpeed == 0.0) {
    return false;
  }
  // マックスアウトの割合が0.0〜1.0の範囲外の場合は開始しない
  if(maxoutRate < 0.0 || maxoutRate > 1.0) {
    Logger::warning("マックスアウトの割合は0.0〜1.0の範囲で設定してください");
    return false;
  }

  // デッドバンドの割合がマックスアウトの割合を上回る場合は開始しない
  if(deadbandRate < 0.0 || deadbandRate > maxoutRate) {
    Logger::warning("デッドバンドの割合は0.0以上かつマックスアウトの割合以下で設定してください");
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

  anglePid.prepare();
}
void Straight::executeStep()
{
  double requiredRightPower = speedCalculator.calculateRightMotorPower();
  double requiredLeftPower = speedCalculator.calculateLeftMotorPower();
  double turningPower = 0.0;

  if(shouldUseIMU) {
    // 現在角度を取得
    double currentAngle = robot.getIMUControllerInstance().getAzimuth();

    // 目標角度との差を計算
    double angleDeviation = targetAngle - currentAngle;

    // -180〜180度に正規化
    angleDeviation = AngleNormalizer::normalizeAngle(angleDeviation);

    // PID制御で旋回量を計算
    turningPower = anglePid.calculatePid(angleDeviation);

    // デッドバンドとマックスアウトを計算
    double deadbandPower = deadbandRate * std::abs(targetSpeed);
    double maxoutPower = maxoutRate * std::abs(targetSpeed);

    // デッドバンド
    if(std::abs(turningPower) < deadbandPower) {
      turningPower = 0.0;
    } else {
      // マックスアウト
      turningPower = std::min(std::max(turningPower, -maxoutPower), maxoutPower);
    }
  }

  // モーターにPower値をセット
  robot.getWheelMotorControllerInstance().setRightPower(requiredRightPower + turningPower);

  robot.getWheelMotorControllerInstance().setLeftPower(requiredLeftPower - turningPower);
}
void Straight::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}