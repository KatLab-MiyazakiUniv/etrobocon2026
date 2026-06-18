/**
 * @file   AngleCondition.cpp
 * @brief  目標角度との誤差に基づいて動作を継続するかを判定するクラス
 * @author okuyama0528 yutaro-1214
 */
#include "AngleCondition.h"

AngleCondition::AngleCondition(Robot& robot, double _targetAngle, double _tolerance)
  : BaseContinuationCondition(robot), targetAngle(_targetAngle), tolerance(_tolerance)
{
}

// 動作を継続するかどうかを判定する関数
bool AngleCondition::shouldContinue()
{
  double currentAngle
      = robot.getIMUControllerInstance().getAzimuth();  // IMUから現在の角度（方位角）を取得

  // 目標角度との差を計算し、-180〜180の範囲に正規化
  double error = AngleNormalizer::normalizeAngle(targetAngle - currentAngle);

  return std::fabs(error) > tolerance;
}

double AngleCondition::getTargetAngle() const
{
  return targetAngle;
}