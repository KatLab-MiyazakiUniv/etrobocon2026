/**
 * @file   QRAbsoluteRotation.cpp
 * @brief  絶対角度で回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#include "QRAbsoluteRotation.h"
#include <algorithm>
#include <cmath>

QRAbsoluteRotation::QRAbsoluteRotation(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    const Pid::PidGain& _anglePidGain, double _targetAbsAngle,
    CameraServer::QrCodeDetectorRequest& _qrDetectionRequest, bool _decryptRequired)
  : Rotation(_robot, std::move(_continuationCondition), _anglePidGain),
    anglePidGain(_anglePidGain),
    targetAbsAngle(_targetAbsAngle),
    qrDetectionRequest(_qrDetectionRequest),
    decryptRequired(_decryptRequired)
{
  LOG_CREATE("QRAbsoluteRotation");
}

QRAbsoluteRotation::~QRAbsoluteRotation()
{
  LOG_DESTROY("QRAbsoluteRotation");
}

void QRAbsoluteRotation::prepare()
{
  targetAngle = targetAbsAngle;
  anglePid = Pid(anglePidGain.kp, anglePidGain.ki, anglePidGain.kd, 0.0);
  Rotation::prepare();
}

void QRAbsoluteRotation::executeStep()
{
  // QR未検出なら検出処理を行う
  if(decryptRequired) {
    if(!robot.getHaveQR2contents()) {
      // 検出処理
      GatePosition::setGatePosition(robot, qrDetectionRequest, decryptRequired);
    }
  } else {
    if(!robot.getHaveQR1contents()) {
      // 検出処理
      GatePosition::setGatePosition(robot, qrDetectionRequest, decryptRequired);
    }
  }

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
