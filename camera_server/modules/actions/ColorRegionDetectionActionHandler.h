/**
 * @file ColorRegionDetectionActionHandler.h
 * @brief 色領域をするクラス
 * @author sadomiya-sousi
 */

#ifndef COLOR_REGION_DETECTION_ACTION_HANDLER_H
#define COLOR_REGION_DETECTION_ACTION_HANDLER_H

#include "SystemInfo.h"
#include "CameraCapture.h"
#include "SocketProtocol.h"
#include "ColorRegionDetector.h"

class ColorRegionDetectionActionHandler {
 public:
  /**
   * @brief コンストラクタ
   * @param camera カメラキャプチャのインスタンス
   */
  ColorRegionDetectionActionHandler(CameraCapture& camera);

  /**
   * @brief ライン検出アクションを実行する
   * @param request クライアントからのリクエスト
   * @param response クライアントへのレスポンス
   */
  void execute(const CameraServer::ColorRegionDetectorRequest& request,
               CameraServer::ColorRegionDetectorResponse& response);

 private:
  CameraCapture& camera;
  ColorRegionDetector detector;
};

#endif  // COLOR_REGION_DETECTION_ACTION_HANDLER_H
