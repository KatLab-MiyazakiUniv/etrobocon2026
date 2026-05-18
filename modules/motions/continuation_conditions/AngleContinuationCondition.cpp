#include "AngleContinuationCondition.h"
#include <cmath>

AngleContinuationCondition::AngleContinuationCondition(Robot& robot, double _tolerance)
  : BaseContinuationCondition(robot), targetAngle(0.0), tolerance(_tolerance)
{
}

bool AngleContinuationCondition::shouldContinue()
{
  double currentAngle = robot.getIMUControllerInstance().getAzimuth();
  double error = normalizeAngle(targetAngle - currentAngle);
  return std::fabs(error) > tolerance;
}

double AngleContinuationCondition::normalizeAngle(double angle)
{
  while(angle > 180.0) angle -= 360.0;
  while(angle < -180.0) angle += 360.0;
  return angle;
}
