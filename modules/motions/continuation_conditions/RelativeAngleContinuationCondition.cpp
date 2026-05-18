/**
 * @file   RelativeAngleContinuationCondition.cpp
 * @brief  現在角度を基準に、指定した相対角度に到達したかどうかで動作の継続/終了を判断するクラス
 * @author okuyama0528 yutaro-1214
 */
#include "RelativeAngleContinuationCondition.h"

RelativeAngleContinuationCondition::RelativeAngleContinuationCondition(Robot& robot,
                                                                       double _deltaAngle,
                                                                       double _tolerance)
  : AngleContinuationCondition(robot, _tolerance), deltaAngle(_deltaAngle)
{
}

void RelativeAngleContinuationCondition::prepare()
{
  // IMUから現在の方位角（絶対角度）を取得
  double initialAngle = robot.getIMUControllerInstance().getAzimuth();
  // 現在角度 + 相対角度 = 目標角度
  // このtargetAngleに到達したかどうかをもとに、動作の継続/終了が判定される
  targetAngle = initialAngle + deltaAngle;
}
