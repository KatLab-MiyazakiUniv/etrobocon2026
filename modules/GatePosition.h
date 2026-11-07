/**
 * @file   GatePosition.h
 * @brief  QRコードからゲート位置情報を取得するクラス
 * @author migaku2645
 */

#ifndef GATE_POSITION_H
#define GATE_POSITION_H

#include "SocketProtocol.h"
#include "GatePositionParser.h"
#include "Logger.h"
#include "Decrypter.h"

class GatePosition {
 public:
  /**
   * @brief QR検出、復号、解析、ゲート位置情報の設定を行う
   * @param robot ロボットインスタンス
   * @param qrDetectionRequest QRコード検出リクエスト
   * @param decryptRequired 復号するか
   */
  static void setGatePosition(Robot& robot,
                              const CameraServer::QrCodeDetectorRequest& qrDetectionRequest,
                              bool decryptRequired);

 private:
  // インスタンス化禁止
  GatePosition() = delete;
  ~GatePosition() = delete;
};

#endif