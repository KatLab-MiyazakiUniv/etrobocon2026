/**
 * @file   LineTrace.cpp
 * @brief  ライントレース動作を実行するクラス
 * @author okuyama0528
 */

#include "LineTrace.h"

LineTrace::LineTrace(Robot& _robot,
                     std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                     double _targetSpeed, int _targetBrightness, const Pid::PidGain& _pidGain)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetBrightness(_targetBrightness),
    pidGain(_pidGain),
    speedCalculator(_robot, _targetSpeed)
{
  LOG_CREATE("LineTrace");
}

LineTrace::~LineTrace()
{
  LOG_DESTROY("LineTrace");
}

void LineTrace::prepare()
{
  // Robotから取得したコース設定に基づきエッジを決定
  edgeSign = (robot.getEdge() == Edge::LeftEdge) ? -1 : 1;
}

void LineTrace::executeStep()
{
  Pid pid(pidGain.kp, pidGain.ki, pidGain.kd, targetBrightness);

  // 基本Speed値を計算
  double baseRightPower = speedCalculator.calculateRightMotorPower();
  double baseLeftPower = speedCalculator.calculateLeftMotorPower();

  // PIDで旋回値を計算
  double turningPower
      = pid.calculatePid(robot.getColorSensorControllerInstance().getReflectance()) * edgeSign;

  // モータのPower値をセット（前進の時0を下回らないように，後進の時0を上回らないようにセット）
  double rightPower = baseRightPower > 0.0 ? std::max(baseRightPower - turningPower, 0.0)
                                           : std::min(baseRightPower + turningPower, 0.0);
  double leftPower = baseLeftPower > 0.0 ? std::max(baseLeftPower + turningPower, 0.0)
                                         : std::min(baseLeftPower - turningPower, 0.0);

  robot.getWheelMotorControllerInstance().setRightPower(rightPower);
  robot.getWheelMotorControllerInstance().setLeftPower(leftPower);
}

void LineTrace::finish()
{
  // モータを停止
  robot.getWheelMotorControllerInstance().stopBoth();
}