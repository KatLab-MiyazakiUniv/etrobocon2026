/**
 * @file   ColorRegionCenterCondition.cpp
 * @brief  色領域検出結果の中心座標が指定範囲内に入るまで継続する条件クラス
 */

#include "ColorRegionCenterCondition.h"

ColorRegionCenterCondition::ColorRegionCenterCondition(
    Robot& _robot, const CameraServer::ColorRegionDetectorRequest& _colorDetectionRequest,
    double _targetCenterX, double _toleranceX, int _consecutiveCountThreshold)
  : BaseContinuationCondition(_robot),
    colorDetectionRequest(_colorDetectionRequest),
    targetCenterX(_targetCenterX),
    toleranceX(std::abs(_toleranceX)),
    consecutiveCountThreshold(_consecutiveCountThreshold),
    inRangeCount(0)
{
  LOG_CREATE("ColorRegionCenterCondition");
}

ColorRegionCenterCondition::~ColorRegionCenterCondition()
{
  LOG_DESTROY("ColorRegionCenterCondition");
}

void ColorRegionCenterCondition::prepare()
{
  inRangeCount = 0;
}

bool ColorRegionCenterCondition::shouldContinue()
{
  SocketClient& client = robot.getCameraSocketClientInstance();
  CameraServer::ColorRegionDetectorResponse response;

  bool success = client.executeColorRegionDetection(colorDetectionRequest, response);

  // 通信失敗または対象色が検出されない場合は継続
  if(!success || !response.result.wasDetected) {
    inRangeCount = 0;
    return true;
  }

  // バウンディングボックスの中心X座標を計算
  double currentCenterX = (response.result.topLeft.x + response.result.bottomRight.x) / 2.0;
  double diff = std::abs(currentCenterX - targetCenterX);

  // 目標値 ± toleranceX の範囲内か判定
  if(diff <= toleranceX) {
    inRangeCount++;
    Logger::printfLog(
        Logger::DEBUG,
        "ColorRegionCenterCondition: 範囲内検出 (%d/%d) [CenterX: %.1f, Target: %.1f, Diff: %.1f]",
        inRangeCount, consecutiveCountThreshold, currentCenterX, targetCenterX, diff);
  } else {
    inRangeCount = 0;
  }

  // 規定回数連続で範囲内に入ったら終了 (shouldContinue = false)
  if(inRangeCount >= consecutiveCountThreshold) {
    Logger::info("ColorRegionCenterCondition: 目標範囲内に到達したため終了します");
    return false;
  }

  return true;
}

double ColorRegionCenterCondition::getTargetCenterX() const
{
  return targetCenterX;
}

double ColorRegionCenterCondition::getToleranceX() const
{
  return toleranceX;
}
