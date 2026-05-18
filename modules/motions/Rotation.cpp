/**
 * @file   Rotation.cpp
 * @brief  その場回転動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "Rotation.h"
#include <cmath>
#include <algorithm>

/**
 * @brief コンストラクタ
 * @param _robot                 ロボット本体への参照
 * @param _continuationCondition 動作継続条件
 * @param _anglePidGain          回頭制御用PIDゲイン
 */
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
 * @brief 現在の機体角度を取得する
 * @return IMUから取得した現在の方位角
 */
double Rotation::getCurrentAngle()
{
  return robot.getIMUControllerInstance().getAzimuth();
}

/**
 * @brief 回頭動作を1ステップ実行する
 *
 * 現在角度と目標角度の偏差を計算し、
 * PID制御によって左右モータ出力を決定する。
 */
void Rotation::executeStep()
{
  // 現在の機体角度を取得
  double currentAngle = getCurrentAngle();

  // 目標角度との差を計算し、-180～180度に正規化

  double error = normalizeAngle(targetAngle - currentAngle);

  // PID制御によって旋回量を計算
  double turn = anglePid.calculatePid(error, 0.0);

  // その場回転を行うため左右モータへ逆方向の出力を設定
  currentRightPower = turn;
  currentLeftPower = -turn;

  // モータ出力を適用
  robot.getWheelMotorControllerInstance().setRightPower(currentRightPower);
  robot.getWheelMotorControllerInstance().setLeftPower(currentLeftPower);
}

/**
 * @brief 回頭動作終了時の処理
 *
 * 左右モータを停止する。
 */
void Rotation::finish()
{
  robot.getWheelMotorControllerInstance().setRightPower(0);
  robot.getWheelMotorControllerInstance().setLeftPower(0);
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
