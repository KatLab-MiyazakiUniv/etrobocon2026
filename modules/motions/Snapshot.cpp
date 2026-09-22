/**
 * @file   Snapshot.cpp
 * @brief  サーバーに写真撮影を依頼するクラス
 * @author sadomiya-sousi
 */

#include "Snapshot.h"

Snapshot::Snapshot(Robot& _robot, const std::string& _fileName,
                   const CameraServer::QrCodeDetectorRequest& _qrDetectionRequest,
                   std::unique_ptr<BaseContinuationCondition> continuationCondition)
  : BaseMotion(_robot, std::move(continuationCondition)),
    fileName(_fileName),
    qrDetectionRequest(_qrDetectionRequest)

{
  LOG_CREATE("Snapshot");
}
Snapshot::~Snapshot()
{
  LOG_DESTROY("Snapshot");
}

void Snapshot::executeStep()
{
  Logger::printfLog(Logger::INFO, "Requesting snapshot: %s", fileName.c_str());

  CameraServer::SnapshotActionRequest snapshotRequest;
  snapshotRequest.command = CameraServer::Command::SNAPSHOT;

  strncpy(snapshotRequest.fileName, fileName.c_str(), sizeof(snapshotRequest.fileName) - 1);
  snapshotRequest.fileName[sizeof(snapshotRequest.fileName) - 1] = '\0';

  CameraServer::SnapshotActionResponse snapshotResponse;

  bool snapshotSuccess = robot.getCameraSocketClientInstance().executeSnapshotAction(
      snapshotRequest, snapshotResponse);

  if(snapshotSuccess) {
    Logger::info("Snapshot:撮影成功");
  } else {
    Logger::error("Snapshot:撮影失敗");
  }

  // 検出処理の呼び出し
  SocketClient& client = robot.getCameraSocketClientInstance();
  bool qrSuccess = false;
  bool wasDetected = false;

  CameraServer::QrCodeDetectorResponse qrResponse;
  qrSuccess = client.executeQrCodeDetection(qrDetectionRequest, qrResponse);
  wasDetected = qrResponse.wasDetected;

  if(!qrSuccess) {
    Logger::warning("CameraTracking:通信に失敗しました");
    return;
  }

  if(!wasDetected) {
    Logger::warning("CameraTracking:検出対象が検出できませんでした");
    return;
  }
}