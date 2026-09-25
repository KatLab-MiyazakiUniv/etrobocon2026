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
  // QR検出済みならreturn
  if(decryptRequired) {
    if(getHaveQR2contents()) {
      return;
    }
  } else {
    if(getHaveQR1contents()) {
      return;
    }
  }

  // QRコードを検出

  SocketClient& client = robot.getCameraSocketClientInstance();
  bool success = false;
  bool wasDetected = false;

  CameraServer::QrCodeDetectorResponse qrResponse;
  success = client.executeQrCodeDetection(qrDetectionRequest, qrResponse);
  wasDetected = qrResponse.wasDetected;

  if(!wasDetected || !success) {
    Logger::error("GatePositionDetection: QRコードの検出に失敗しました。");
    return;
  } else {
    Logger::error("GatePositionDetection: QRコードの検出に成功しました。");

    // QR検出成功ならフラグを更新
    if(decryptRequired) {
      setHaveQR2contents()
    } else {
      setHaveQR1contents()
    }
  }

  //  QRコードの暗号文を復号
  std::string gatePositionData;

  if(decryptRequired) {
    // AesDecryptor decryptor(std::string(robot.getDecryptionKey()), qrResponse.content);
    // 青・黄色：復号する
    gatePositionData
        = Decrypter::decrypt(std::string(robot.getDecryptionKey()), qrResponse.content);

    if(gatePositionData.empty()) {
      Logger::error("GatePositionDetection: QRコードの復号に失敗しました。");
      return;
    }
  } else {
    // 赤：復号しない
    gatePositionData = qrResponse.content;
  }

  //  平文を解析して,ゲート位置情報をRobotに設定

  GatePositionParser parser(gatePositionData, robot);

  if(!parser.parse()) {
    Logger::error("GatePositionDetection: ゲート位置情報の解析に失敗しました。");
    return;
  }

  Logger::info("GatePositionDetection: ゲート位置情報の設定に成功しました。");
}