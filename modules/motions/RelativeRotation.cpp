/**
 * @file   RelativeRotation.cpp
 * @brief  相対角度で回頭動作を実行するクラス
 * @author okuyama0528　yutaro-1214
 */
#include "RelativeRotation.h"

RelativeRotation::RelativeRotation(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    const Pid::PidGain& _anglePidGain, double _relativeAngle)
  : Rotation(_robot, std::move(_continuationCondition), _anglePidGain),
    anglePidGain(_anglePidGain),
    relativeTargetAngle(_relativeAngle)
{
  LOG_CREATE("RelativeRotation");
}

RelativeRotation::~RelativeRotation()
{
  LOG_DESTROY("RelativeRotation");
}

void RelativeRotation::prepare()
{
  double initialAngle = getCurrentAngle();  // 現在の走行体角度を取得する

  // 現在角度にrelativeAngleを加算し、-180～180度に正規化して目標角度を算出する
  targetAngle = AngleNormalizer::normalizeAngle(initialAngle + relativeTargetAngle);

  anglePid = Pid(anglePidGain.kp, anglePidGain.ki, anglePidGain.kd, 0.0);
  Rotation::prepare();
}

void RelativeRotation::executeStep()
{
  double error = AngleNormalizer::normalizeAngle(targetAngle - getCurrentAngle());
  double turn = anglePid.calculatePid(error);
  // 整数化で0になったり、静止摩擦を越えられない出力になるのを防ぐ。
  // PIDはtarget(0)-errorを計算するため、目標方向の符号は-error。
  if(error == 0.0) {
    turn = 0.0;
  } else if(std::fabs(turn) < MIN_TURN_POWER) {
    turn = std::copysign(MIN_TURN_POWER, -error);
  }
  turn = std::clamp(turn, -MAX_TURN_POWER, MAX_TURN_POWER);
  robot.getWheelMotorControllerInstance().setRightPower(turn);
  robot.getWheelMotorControllerInstance().setLeftPower(-turn);
}