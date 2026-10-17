/**
 * @file   GatePositionDetection.h
 * @brief  QRコードからゲート位置情報を取得するクラス
 * @author migaku2645
 */

#ifndef GATE_POSITION_H
#define GATE_POSITION_H

#include "SocketProtocol.h"
#include <string>
#include "GatePositionParser.h"
#include "Logger.h"
#include "Decrypter.h"

class GatePosition {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot ロボットインスタンス
   * @param _decryptRequired 復号するか
   * @param _qrDetectionRequest QRコード検出リクエスト
   */
  GatePosition(Robot& _robot, const CameraServer::QrCodeDetectorRequest& _qrDetectionRequest,
               bool _decryptRequired);

  /**
   * @brief デストラクタ
   */
  ~GatePosition();

  /**
   * @brief QR検出、平文の解析、ゲート位置情報設定を一回行う
   */
  void setGatePosition();

 private:
  Robot& robot;                                            // Robot インスタンスの参照
  CameraServer::QrCodeDetectorRequest qrDetectionRequest;  // QRコード検出リクエスト
  bool decryptRequired = false;
};

#endif