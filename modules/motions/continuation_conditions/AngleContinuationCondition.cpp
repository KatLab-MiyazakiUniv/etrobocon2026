#include "AngleContinuationCondition.h"
#include <cmath>

AngleContinuationCondition::AngleContinuationCondition(Robot& robot, double _targetAngle,
                                                       double _tolerance)
  : BaseContinuationCondition(robot),
    targetAngle(_targetAngle),
    tolerance(_tolerance),
    initialAngle(0.0),
    currentAngle(0.0)
{
}

void AngleContinuationCondition::prepare()
{
  // スタート角度（ログ用・デバッグ用）
  initialAngle = robot.getIMUControllerInstance().getAzimuth();
}

bool AngleContinuationCondition::shouldContinue()
{
  currentAngle = robot.getIMUControllerInstance().getAzimuth();

  double error = normalizeAngle(targetAngle - currentAngle);

  // まだ誤差が大きいなら継続
  return std::fabs(error) > tolerance;
}

double AngleContinuationCondition::normalizeAngle(double angle)
{
  while(angle > 180.0) angle -= 360.0;
  while(angle < -180.0) angle += 360.0;
  return angle;
}