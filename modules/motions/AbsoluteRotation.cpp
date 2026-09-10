/**
 * @file   AbsoluteRotation.cpp
 * @brief  絶対角度で回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#include "AbsoluteRotation.h"
#include <algorithm>
#include <cmath>

AbsoluteRotation::AbsoluteRotation(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    const Pid::PidGain& _anglePidGain, double _targetAbsAngle)
  : Rotation(_robot, std::move(_continuationCondition), _anglePidGain),
    anglePidGain(_anglePidGain),
    targetAbsAngle(_targetAbsAngle)
{
  LOG_CREATE("AbsoluteRotation");
}

AbsoluteRotation::~AbsoluteRotation()
{
  LOG_DESTROY("AbsoluteRotation");
}

void AbsoluteRotation::prepare()
{
  targetAngle = targetAbsAngle;
  anglePid = Pid(anglePidGain.kp, anglePidGain.ki, anglePidGain.kd, 0.0);
  Rotation::prepare();
}

void AbsoluteRotation::executeStep()
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
