/**
 * @file   ColorRegionCenterCondition.cpp
 * @brief  色領域検出結果の中心座標が指定範囲内に入るまで継続する条件クラス
 */

#include "ColorRegionCenterCondition.h"

ColorRegionCenterCondition::ColorRegionCenterCondition(
    Robot& _robot, const CameraServer::ColorRegionDetectorRequest& _colorDetectionRequest,
    double _targetCenterY, int _consecutiveCountThreshold)
  : BaseContinuationCondition(_robot),
    colorDetectionRequest(_colorDetectionRequest),
    targetCenterY(_targetCenterY),
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

  // バウンディングボックスの中心Y座標を計算
  double currentCenterY = (response.result.topLeft.y + response.result.bottomRight.y) / 2.0;

  // 目標値 ± toleranceY の範囲内か判定
  if(currentCenterY > targetCenterY) {
    inRangeCount++;
    Logger::printfLog(
        Logger::DEBUG,
        "ColorRegionCenterCondition: 範囲内検出 (%d/%d) [CenterX: %.1f, Target: %.1f]",
        inRangeCount, consecutiveCountThreshold, currentCenterY, targetCenterY);
  } else {
    inRangeCount = 0;
    Logger::printfLog(
        Logger::DEBUG,
        "ColorRegionCenterCondition: 失敗");

  }

  // 規定回数連続で範囲内に入ったら終了 (shouldContinue = false)
  if(inRangeCount >= consecutiveCountThreshold) {
    Logger::info("ColorRegionCenterCondition: 目標範囲内に到達したため終了します");
    return false;
  }

  return true;
}

double ColorRegionCenterCondition::getTargetCenterY() const
{
  return targetCenterY;
}


