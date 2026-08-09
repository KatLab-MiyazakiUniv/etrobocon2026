/**
 * @file QrCodeDetectionActionHandler.h
 * @brief QRコードを検出するクラス
 * @author HaruArima08
 */

#ifndef QR_CODE_DETECTION_ACTION_HANDLER_H
#define QR_CODE_DETECTION_ACTION_HANDLER_H

#include "SystemInfo.h"
#include "CameraCapture.h"
#include "SocketProtocol.h"
#include "QrCodeDetector.h"
#include <cmath>
#include <cstring>

class QrCodeDetectionActionHandler {
 public:
  /**
   * @brief コンストラクタ
   * @param camera カメラキャプチャのインスタンス
   */
  QrCodeDetectionActionHandler(CameraCapture& camera);

  /**
   * @brief デストラクタ
   */
  ~QrCodeDetectionActionHandler();

  /**
   * @brief QRコード検出アクションを実行する
   * @param request クライアントからのリクエスト
   * @param response クライアントへのレスポンス
   */
  void execute(const CameraServer::QrCodeDetectorRequest& request,
               CameraServer::QrCodeDetectorResponse& response);

  /**
   * @brief カメラキャプチャのインスタンスを取得する
   * @return CameraCapture& カメラキャプチャのインスタンスへの参照
   */
  const CameraCapture& getCamera() const;

 private:
  CameraCapture& camera;    // カメラキャプチャのインスタンスへの参照
  QrCodeDetector detector;  // QRコード検出器
};

#endif  // QR_CODE_DETECTION_ACTION_HANDLER_H
