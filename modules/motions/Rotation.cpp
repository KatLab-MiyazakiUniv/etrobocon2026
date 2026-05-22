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
    anglePid(_anglePidGain.kp, _anglePidGain.ki, _anglePidGain.kd, 0.0),
    currentRightPower(0),
    currentLeftPower(0)
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
  // 現在の方位角を取得
  double currentAngle = getCurrentAngle();

  // 目標角度との差を計算し、-180～180度に正規化
  double error = normalizeAngle(targetAngle - currentAngle);

  // 旋回量を計算
  double turn = anglePid.calculatePid(error, 0.0);

  // モータにPower値をセット
  robot.getWheelMotorControllerInstance().setRightPower(turn);
  robot.getWheelMotorControllerInstance().setLeftPower(-turn);
}

/**
 * @brief 回頭動作終了時に左右モータを停止するための処理
 */
void Rotation::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
  robot.getWheelMotorControllerInstance().resetBothPower();
}

/**
 * @brief 現在の機体角度を取得する
 * @return IMUから取得した現在の方位角
 */
double Rotation::getCurrentAngle()
{
  return robot.getIMUControllerInstance().getAzimuth();
}

/**
 * @brief 角度を-180～180度の範囲に正規化する
 * @param angle 正規化前の角度
 * @return 正規化後の角度
 */
double Rotation::normalizeAngle(double angle)
{
  // 180度を超える場合は360度引く
  while(angle > 180.0) angle -= 360.0;

  // -180度未満の場合は360度足す
  while(angle < -180.0) angle += 360.0;
  return angle;
}
