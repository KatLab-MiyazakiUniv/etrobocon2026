/**
 * @file   Rotation.cpp
 * @brief  IMU方位角を用いたその場回転動作を実行するクラス
 * @author yutaro-1214
 */

#include "Rotation.h"

Rotation::Rotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   float _targetTurnPower)
  : BaseMotion(_robot, std::move(_continuationCondition)), targetTurnPower(_targetTurnPower)
{
}

void Rotation::executeStep()
{
  // その場回転
  // 正値:
  //   右車輪前進
  //   左車輪後退
  //
  // 負値:
  //   右車輪後退
  //   左車輪前進

  robot.getWheelMotorControllerInstance().setRightPower(targetTurnPower);

  robot.getWheelMotorControllerInstance().setLeftPower(-targetTurnPower);
}

void Rotation::finish()
{
  // 両モータ停止
  robot.getWheelMotorControllerInstance().stopBoth();
}