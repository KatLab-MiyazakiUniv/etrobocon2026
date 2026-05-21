/**
 * @file   RelativeAngleContinuationCondition.cpp
 * @brief  現在角度を基準に、指定した相対角度に到達したかどうかで動作を継続すべきかを判定するクラス
 * @author okuyama0528 yutaro-1214
 */
#include "RelativeAngleContinuationCondition.h"

RelativeAngleContinuationCondition::RelativeAngleContinuationCondition(Robot& robot,
                                                                       double _relativeAngle,
                                                                       double _tolerance)
  : AngleContinuationCondition(robot, _tolerance), relativeAngle(_relativeAngle)
{
}

void RelativeAngleContinuationCondition::prepare()
{
  double initialAngle
      = robot.getIMUControllerInstance().getAzimuth();  // IMUから現在の方位角（絶対角度）を取得

  /**
   * @brief 目標角度の設定
   * @details
   * 現在角度に相対角度(relativeAngle)を加算して目標角度(targetAngle)を算出する。
   * このtargetAngleを基準に、動作を継続すべきかを判定される。
   */
  targetAngle = initialAngle + relativeAngle;
}
