/**
 * @file SquareDetectionActionHandler.h
 * @brief 正方形検出アクションを処理するクラス
 * @author okuyama0528
 */

#ifndef SQUARE_DETECTION_ACTION_HANDLER_H
#define SQUARE_DETECTION_ACTION_HANDLER_H

#include "SystemInfo.h"
#include "CameraCapture.h"
#include "SocketProtocol.h"
#include "QrCodeDetector.h"
#include "SquareDetector.h"
#include <cmath>
#include <cstring>

class SquareDetectionActionHandler {
 public:
  /**
   * @brief コンストラクタ
   * @param camera カメラキャプチャのインスタンス
   */
  SquareDetectionActionHandler(CameraCapture& camera);

  /**
   * @brief デストラクタ
   */
  ~SquareDetectionActionHandler();

  /**
   * @brief 正方形検出アクションを実行する
   * @param request クライアントからのリクエスト
   * @param response クライアントへのレスポンス
   */
  void execute(const CameraServer::SquareDetectorRequest& request,
               CameraServer::SquareDetectorResponse& response);

 private:
  CameraCapture& camera;    // カメラキャプチャのインスタンスへの参照
  SquareDetector detector;  // 正方形検出器
};
#endif  // SQUARE_DETECTION_ACTION_HANDLER_H