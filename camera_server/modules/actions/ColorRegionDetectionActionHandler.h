/**
 * @file ColorRegionDetectionActionHandler.h
 * @brief 色領域をするクラス
 * @author sadomiya-sousi
 */

#ifndef COLOR_REGION_DETECTION_ACTION_HANDLER_H
#define COLOR_REGION_DETECTION_ACTION_HANDLER_H

#include "CameraCapture.h"
#include "SocketProtocol.h"
#include "ColorRegionDetector.h"
#include <memory>

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
  CameraCapture& camera;                                   // カメラキャプチャーインスタンス
  std::unique_ptr<ColorRegionDetector> detector;           // 使い回すインスタンスを保持
  bool hasCachedRequest = false;                           // 1つ前のRequestを保存しているかどうか
  CameraServer::ColorRegionDetectorRequest cachedRequest;  // 1つ前のRequestをキャッシュ
};

#endif  // TWO_COLOR_LINE_DETECTION_ACTION_HANDLER_H
