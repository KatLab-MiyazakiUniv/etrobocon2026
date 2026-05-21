/**
 * @file   AngleContinuationCondition.cpp
 * @brief  目標角度との誤差に基づいて動作を継続するかを判定するクラス
 * @author okuyama0528 yutaro-1214
 */
#include "AngleContinuationCondition.h"

AngleContinuationCondition::AngleContinuationCondition(Robot& robot, double _tolerance)
  : BaseContinuationCondition(robot), targetAngle(0.0), tolerance(_tolerance)
{
}

// 動作を継続するかどうかを判定する関数
bool AngleContinuationCondition::shouldContinue()
{
  double currentAngle
      = robot.getIMUControllerInstance().getAzimuth();  // IMUから現在の角度（方位角）を取得

  // 目標角度との差を計算し、-180〜180の範囲に正規化
  double error = normalizeAngle(targetAngle - currentAngle);

  /**
   * @brief 誤差に基づく継続判定
   * @details
   * 目標角度との誤差が許容範囲を超えている場合は「未到達」として動作を継続し、
   * 許容範囲内であれば「到達」とみなして動作を終了する
   */
  return std::fabs(error) > tolerance;
}

// 角度を -180〜180 の範囲に収める関数
double AngleContinuationCondition::normalizeAngle(double angle)
{
  // 180度を超えた場合は360度引く
  while(angle > 180.0) angle -= 360.0;

  // -180度未満の場合は360度足す
  while(angle < -180.0) angle += 360.0;
  return angle;
}
