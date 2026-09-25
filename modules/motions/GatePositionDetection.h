/**
 * @file   GatePositionDetection.h
 * @brief  QRコードからゲート位置情報を取得する動作クラス
 * @author migaku2645
 */

#ifndef GATE_POSITION_DETECTION_H
#define GATE_POSITION_DETECTION_H

#include "BaseMotion.h"
#include "SocketProtocol.h"
#include <string>
#include "GatePositionParser.h"
#include "Logger.h"
#include "Decrypter.h"

class GatePositionDetection : public BaseMotion {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot ロボットインスタンス
   * @param _decryptRequired 復号するか
   * @param _qrDetectionRequest QRコード検出リクエスト
   * @param continuationCondition 継続条件
   */
  GatePositionDetection(Robot& _robot,
                        std::unique_ptr<BaseContinuationCondition> continuationCondition,
                        const CameraServer::QrCodeDetectorRequest& _qrDetectionRequest,
                        bool _decryptRequired);

  /**
   * @brief デストラクタ
   */
  ~GatePositionDetection();

 protected:
  /**
   * @brief 写真撮影からゲート位置情報設定までを実行する
   */
  void executeStep() override;

 private:
  CameraServer::QrCodeDetectorRequest qrDetectionRequest;  // QRコード検出リクエスト
  bool decryptRequired = false;
};

#endif