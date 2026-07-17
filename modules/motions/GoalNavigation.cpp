/**
 * @file   GoalNavigation.cpp
 * @brief 目標座標まで移動するクラス
 * @author yutaro-1214
 */

#include "GoalNavigation.h"

#include <cmath>

GoalNavigation::GoalNavigation(Robot& _robot,
                               std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                               double _goalX, double _goalY, double _targetSpeed,
                               const Pid::PidGain& _rightPid, const Pid::PidGain& _leftPid,
                               const Pid::PidGain& _anglePidGain)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    state(State::ROTATE),
    goalX(_goalX),
    goalY(_goalY),
    targetSpeed(_targetSpeed),
    anglePid(_anglePidGain.kp, _anglePidGain.ki, _anglePidGain.kd, 0.0),
    speedCalculator(_robot, _rightPid, _leftPid, _targetSpeed),
    targetAngle(0.0)
{
  LOG_CREATE("GoalNavigation");
}

GoalNavigation::~GoalNavigation()
{
  LOG_DESTROY("GoalNavigation");
}

bool GoalNavigation::canStart()
{
  if(targetSpeed == 0.0) {
    return false;
  }

  return true;
}

void GoalNavigation::prepare()
{
  state = State::ROTATE;

  robot.getOdometry().reset();

  robot.getOdometry().initialize(robot.getWheelMotorControllerInstance().getLeftCount(),
                                 robot.getWheelMotorControllerInstance().getRightCount());

  targetAngle = robot.getNavigator().calculateHeading(goalX, goalY);
}

void GoalNavigation::executeStep()
{
  // オドメトリ更新
  robot.getOdometry().update(robot.getWheelMotorControllerInstance().getLeftCount(),
                             robot.getWheelMotorControllerInstance().getRightCount(),
                             robot.getIMUControllerInstance().getAzimuth());

  // 現在位置から目標方向を毎周期更新
  targetAngle = robot.getNavigator().calculateHeading(goalX, goalY);

  double currentAngle = robot.getIMUControllerInstance().getAzimuth();

  double angleDeviation = AngleNormalizer::normalizeAngle(targetAngle - currentAngle);

  switch(state) {
    //--------------------------------------------------
    // 回頭
    //--------------------------------------------------
    case State::ROTATE: {
      double turningPower = anglePid.calculatePid(angleDeviation);

      robot.getWheelMotorControllerInstance().setRightPower(turningPower);
      robot.getWheelMotorControllerInstance().setLeftPower(-turningPower);

      // 十分向けたら直進へ
      if(std::fabs(angleDeviation) <= ANGLE_TOLERANCE) {
        anglePid.reset();  // PIDの内部状態をリセット
        state = State::STRAIGHT;
      }

      break;
    }

    //--------------------------------------------------
    // 直進
    //--------------------------------------------------
    case State::STRAIGHT: {
      double requiredRightPower = speedCalculator.calculateRightMotorPower();

      double requiredLeftPower = speedCalculator.calculateLeftMotorPower();

      double turningPower = anglePid.calculatePid(angleDeviation);

      robot.getWheelMotorControllerInstance().setRightPower(requiredRightPower + turningPower);

      robot.getWheelMotorControllerInstance().setLeftPower(requiredLeftPower - turningPower);

      break;
    }
  }
}

void GoalNavigation::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}