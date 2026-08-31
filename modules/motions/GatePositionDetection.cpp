/**
 * @file   GatePositionDetection.cpp
 * @brief  QRコードからゲート位置情報を取得する動作クラス
 * @author migaku2645
 */

#include "GatePositionDetection.h"

GatePositionDetection::GatePositionDetection(
    Robot& _robot, const std::string& _fileName, const CameraServer::QrCodeDetectorRequest& _qrDetectionRequest,
    std::unique_ptr<BaseContinuationCondition> continuationCondition)
  : BaseMotion(_robot, std::move(continuationCondition)),
    fileName(_fileName),
    qrDetectionRequest(_qrDetectionRequest)
{
  LOG_CREATE("GatePositionDetection");
}

GatePositionDetection::~GatePositionDetection()
{
  LOG_DESTROY("GatePositionDetection");
}

void GatePositionDetection::executeStep()
{
  // -----------------------------
  // 1. 写真撮影
  // -----------------------------

  Snapshot snapshot(robot, fileName, std::make_unique<RepeatCountCondition>(robot, 1));

  snapshot.run();



  // -----------------------------
  // 2. QRコードを検出
  // -----------------------------

  SocketClient& client = robot.getCameraSocketClientInstance();
  bool success = false;
  bool wasDetected = false;

  CameraServer::QrCodeDetectorResponse qrResponse;
  success = client.executeQrCodeDetection(qrDetectionRequest, qrResponse);
  wasDetected = qrResponse.wasDetected;

  if(!wasDetected || !success) {
    Logger::error("GatePositionDetection: QRコードの検出に失敗しました。");
    return;
  }

  // -----------------------------
  // 3. QRコードの暗号文を復号
  // -----------------------------

  AesDecryptor decryptor(std::string(robot.getDecryptionKey()), qrResponse.content);

  std::string plaintext = decryptor.decrypt();

  if(plaintext.empty()) {
    Logger::error("GatePositionDetection: QRコードの復号に失敗しました。");
    return;
  }

  // -----------------------------
  // 4. 平文を解析して
  //    ゲート位置情報をRobotに設定
  // -----------------------------

  GatePositionParser parser(plaintext, robot);

  if(!parser.parse()) {
    Logger::error("GatePositionDetection: ゲート位置情報の解析に失敗しました。");
    return;
  }

  Logger::info("GatePositionDetection: ゲート位置情報の設定に成功しました。");
}