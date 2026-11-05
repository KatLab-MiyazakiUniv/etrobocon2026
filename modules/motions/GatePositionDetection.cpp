/**
 * @file   GatePositionDetection.cpp
 * @brief  QRコードからゲート位置情報を取得する動作クラス
 * @author migaku2645
 */

#include "GatePositionDetection.h"

GatePositionDetection::GatePositionDetection(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> continuationCondition,
    const CameraServer::QrCodeDetectorRequest& _qrDetectionRequest, bool _decryptRequired)
  : BaseMotion(_robot, std::move(continuationCondition)),
    qrDetectionRequest(_qrDetectionRequest),
    decryptRequired(_decryptRequired)
{
  LOG_CREATE("GatePositionDetection");
}

GatePositionDetection::~GatePositionDetection()
{
  LOG_DESTROY("GatePositionDetection");
}

void GatePositionDetection::executeStep()
{
  // 検出処理
  GatePosition gatePosition(robot, qrDetectionRequest, decryptRequired);
  gatePosition.setGatePosition();
}