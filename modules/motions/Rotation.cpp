/**
 * @file   Rotation.cpp
 * @brief  回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "Rotation.h"

Rotation::Rotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   const Pid::PidGain& _anglePidGain)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetAngle(0.0),
    anglePid(_anglePidGain.kp, _anglePidGain.ki, _anglePidGain.kd, 0.0)
{
}

/**
 * @brief 回頭動作を1ステップ実行する
 *
 * 現在角度と目標角度の偏差を計算し、
 * PID制御によって左右モータ出力を決定する。
 */
void Rotation::executeStep()
{
  double currentAngle = getCurrentAngle();  // 現在の方位角を取得

  // 目標角度との差を計算し、-180～180度に正規化
  double error = AngleNormalizer::NormalizeAngle(targetAngle - currentAngle);
  double turn = anglePid.calculatePid(error);  // 旋回量を計算
  // PID制御で回頭方向を決定し、左右モータの出力を算出する
  double requiredRightPower = turn;
  double requiredLeftPower = -(turn);

  // モータにPower値をセット
  robot.getWheelMotorControllerInstance().setRightPower(requiredRightPower);
  robot.getWheelMotorControllerInstance().setLeftPower(requiredLeftPower);
}

void Rotation::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
  robot.getWheelMotorControllerInstance().resetBothPower();
}

double Rotation::getCurrentAngle()
{
  return robot.getIMUControllerInstance().getAzimuth();
}
