/**
 * @file   Rotation.cpp
 * @brief  その場回転動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "Rotation.h"
#include <cmath>
#include <algorithm>

Rotation::Rotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    pid(0.8, 0.0, 0.05, 0.0),
    currentRightPower(0),
    currentLeftPower(0),
    targetAngle(0.0)
{
}

// IMUから現在角度取得
double Rotation::getCurrentAngle()
{
  return robot.getIMUControllerInstance().getAzimuth();
}

// 準備（必要なら派生側でtargetAngleを確定させる）
void Rotation::prepare()
{
  initialAngle = getCurrentAngle();
}

// 1周期ごとの制御
void Rotation::executeStep()
{
  double currentAngle = getCurrentAngle();

  double error = normalizeAngle(targetAngle - currentAngle);

  double turn = pid.calculatePid(error, 0.0);

  // 出力制限
  turn = std::clamp(turn, -60.0, 60.0);

  currentRightPower = -turn;
  currentLeftPower = turn;

  robot.getWheelMotorControllerInstance().setRightPower(currentRightPower);
  robot.getWheelMotorControllerInstance().setLeftPower(currentLeftPower);
}

// 停止処理
void Rotation::finish()
{
  robot.getWheelMotorControllerInstance().setRightPower(0);
  robot.getWheelMotorControllerInstance().setLeftPower(0);
}

// 角度正規化（-180〜180）
double Rotation::normalizeAngle(double angle)
{
  while(angle > 180.0) angle -= 360.0;
  while(angle < -180.0) angle += 360.0;
  return angle;
}