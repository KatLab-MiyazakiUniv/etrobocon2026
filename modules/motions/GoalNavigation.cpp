/**
 * @file   GoalNavigation.cpp
 * @brief 目標座標まで移動するクラス
 * @author yutaro-1214
 */

#include "GoalNavigation.h"

#include <cmath>

GoalNavigation::GoalNavigation(Robot& robot,
                               std::unique_ptr<BaseContinuationCondition> continuationCondition,
                               double goalX, double goalY, double speed,
                               const Pid::PidGain& anglePidGain)
  : BaseMotion(robot, std::move(continuationCondition)),
    state(State::ROTATE),
    goalX(goalX),
    goalY(goalY),
    speed(speed),
    anglePid(anglePidGain.kp, anglePidGain.ki, anglePidGain.kd, 0.0)
{
  LOG_CREATE("GoalNavigation");
}

GoalNavigation::~GoalNavigation()
{
  LOG_DESTROY("GoalNavigation");
}

void GoalNavigation::executeStep()
{
  // 現在位置から目標方位を計算
  double targetHeading = robot.getNavigator().calculateHeading(goalX, goalY);

  // IMUから現在方位を取得
  double currentHeading = robot.getIMUControllerInstance().getAzimuth();

  // 方位誤差
  double error = AngleNormalizer::normalizeAngle(targetHeading - currentHeading);

  switch(state) {
    case State::ROTATE: {
      double turn = anglePid.calculatePid(error);

      robot.getWheelMotorControllerInstance().setRightPower(turn);
      robot.getWheelMotorControllerInstance().setLeftPower(-turn);

      // 目標方位を向いたら直進へ
      if(std::fabs(error) <= ANGLE_TOLERANCE) {
        state = State::STRAIGHT;
        anglePid.reset();
      }

      break;
    }

    case State::STRAIGHT: {
      double turn = anglePid.calculatePid(error);

      robot.getWheelMotorControllerInstance().setRightPower(speed + turn);
      robot.getWheelMotorControllerInstance().setLeftPower(speed - turn);

      break;
    }
  }
}

void GoalNavigation::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
  robot.getWheelMotorControllerInstance().resetBothPower();
}