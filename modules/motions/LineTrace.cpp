/**
 * @file   LineTrace.cpp
 * @brief  ライントレース動作を実行するクラス
 * @author okuyama0528
 */

#include "LineTrace.h"

LineTrace::LineTrace(Robot& _robot,
                     std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                     double _targetSpeed, int _targetBrightness,
                     const Pid::PidGain& _brightnessPidGain, double _deadbandRate,
                     double _maxoutRate, const std::string& _commandId)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetBrightness(_targetBrightness),
    brightnessPid(_brightnessPidGain.kp, _brightnessPidGain.ki, _brightnessPidGain.kd,
                  _targetBrightness),
    speedCalculator(_robot, _targetSpeed),
    deadbandRate(_deadbandRate),
    maxoutRate(_maxoutRate),
    brightnessPidGain(_brightnessPidGain),
    commandId(_commandId)
{
  LOG_CREATE("LineTrace");
}

LineTrace::~LineTrace()
{
  LOG_DESTROY("LineTrace");
}

bool LineTrace::canStart()
{
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

void LineTrace::prepare()
{
  edgeSign = (robot.getEdge() == Edge::LeftEdge) ? -1 : 1;

  brightnessPid.prepare();
}

void LineTrace::executeStep()
{
  // 目標スピードに必要なパワー値を計算
  double baseRightPower = speedCalculator.calculateRightMotorPower();
  double baseLeftPower = speedCalculator.calculateLeftMotorPower();
  double basePower = (std::abs(baseRightPower) + std::abs(baseLeftPower)) / 2.0;

  // PIDで旋回値を計算
  int brightness = robot.getColorSensorControllerInstance().getReflectance();
  double rawTurningPower = brightnessPid.calculatePid(brightness) * edgeSign;
  double turningPower = rawTurningPower;

  // デッドバンドとマックスアウトを適用
  double deadbandPower = deadbandRate * basePower;
  double maxoutPower = maxoutRate * basePower;

  // デッドバンド: 閾値未満の旋回値を無視し、微小な揺れを防ぐ
  if(std::abs(turningPower) < deadbandPower) {
    turningPower = 0.0;
  } else {
    // マックスアウト: 閾値を超える旋回値を制限し、急激な旋回を防ぐ
    turningPower = std::min(std::max(turningPower, -maxoutPower), maxoutPower);
  }

  // ライントレースに必要なPower値を算出（前進の時0を下回らないように，後進の時0を上回らないように計算
  double rightPower = baseRightPower > 0.0 ? std::max(baseRightPower - turningPower, 0.0)
                                           : std::min(baseRightPower + turningPower, 0.0);
  double leftPower = baseLeftPower > 0.0 ? std::max(baseLeftPower + turningPower, 0.0)
                                         : std::min(baseLeftPower - turningPower, 0.0);

  robot.getWheelMotorControllerInstance().setRightPower(rightPower);
  robot.getWheelMotorControllerInstance().setLeftPower(leftPower);

  LogData logData;
  logData.id = commandId.empty() ? "LineTrace" : "LineTrace:" + commandId;
  logData.brightness = brightness;
  logData.rightPower = static_cast<int>(rightPower);
  logData.leftPower = static_cast<int>(leftPower);
  logData.rightSpeed = robot.getWheelMotorControllerInstance().getRightSpeed();
  logData.leftSpeed = robot.getWheelMotorControllerInstance().getLeftSpeed();
  logData.currentVal = brightness;
  logData.target = targetBrightness;
  logData.kp = brightnessPidGain.kp;
  logData.ki = brightnessPidGain.ki;
  logData.kd = brightnessPidGain.kd;
  logData.error = targetBrightness - brightness;
  logData.rawTurn = rawTurningPower;
  logData.turn = turningPower;
  logData.basePower = basePower;
  logData.deadbandPower = deadbandPower;
  logData.maxoutPower = maxoutPower;
  logData.maxoutActive = (maxoutPower > 0.0 && std::abs(rawTurningPower) >= maxoutPower) ? 1 : 0;
  CsvLogger::add(logData);
}

void LineTrace::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}
