/**
 * @file   EtRallyFinish.cpp
 * @brief  ETラリー後のゴール走行を行うクラス
 * @author okuyama0528
 */

#include "EtRallyFinish.h"

namespace {
  constexpr double ROTATION_TOLERANCE = 2.0;
}

EtRallyFinish::EtRallyFinish(Robot& _robot,
                             std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                             const Pid::PidGain& _rotationPid,
                             const Pid::PidGain& _straightAnglePid, double _rotationAngle,
                             double _straightDistance, double _targetSpeed,
                             double _straightDeadbandRate, double _straightMaxoutRate)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    rotationPid(_rotationPid),
    straightAnglePid(_straightAnglePid),
    targetSpeed(_targetSpeed),
    rotationAngle(_rotationAngle),
    straightDistance(_straightDistance),
    straightDeadbandRate(_straightDeadbandRate),
    straightMaxoutRate(_straightMaxoutRate)
{
  LOG_CREATE("EtRallyFinish");
}

void EtRallyFinish::prepare() {}

void EtRallyFinish::executeStep()
{
  Straight straightMotion(robot, std::make_unique<DistanceCondition>(robot, straightDistance),
                          targetSpeed, straightAnglePid, true, straightDeadbandRate,
                          straightMaxoutRate);
  RelativeRotation rotationMotion(
      robot, std::make_unique<RelativeAngleCondition>(robot, rotationAngle, ROTATION_TOLERANCE),
      rotationPid, rotationAngle);
  rotationMotion.run();
  straightMotion.run();
  robot.getWheelMotorControllerInstance().stopBoth();
}

void EtRallyFinish::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}