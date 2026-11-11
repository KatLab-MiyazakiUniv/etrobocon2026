/**
 * @file   GatePosition.cpp
 * @brief  QRコードからゲート位置情報を取得するクラス
 * @author migaku2645
 */

#include "GatePosition.h"

void GatePosition::setGatePosition(Robot& robot,
                                   const CameraServer::QrCodeDetectorRequest& qrDetectionRequest,
                                   bool decryptRequired)
{
  // QR検出済みならreturn
  if(decryptRequired) {
    if(robot.getHaveQR2contents()) {
      return;
    }
  } else {
    if(robot.getHaveQR1contents()) {
      return;
    }
  }

  // QRコードを検出
  SocketClient& client = robot.getCameraSocketClientInstance();

  CameraServer::QrCodeDetectorResponse qrResponse;

  bool success = client.executeQrCodeDetection(qrDetectionRequest, qrResponse);

  if(!success || !qrResponse.wasDetected) {
    Logger::error("GatePositionDetection: QRコードの検出に失敗しました。");
    return;
  }

  Logger::info("GatePositionDetection: QRコードの検出に成功しました。");

  // QRコードの暗号文を復号
  std::string gatePositionData;

  if(decryptRequired) {
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

  // 平文を解析してゲート位置情報をRobotに設定
  GatePositionParser parser(gatePositionData, robot);

  if(!parser.parse()) {
    Logger::error("GatePositionDetection: ゲート位置情報の解析に失敗しました。");
    return;
  }

  Logger::info("GatePositionDetection: ゲート位置情報の設定に成功しました。");

  // QR検出・解析成功ならフラグを更新
  if(decryptRequired) {
    robot.setHaveQR2contents();
  } else {
    robot.setHaveQR1contents();
  }
}