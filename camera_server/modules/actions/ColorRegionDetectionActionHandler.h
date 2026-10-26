/**
 * @file ColorRegionDetectionActionHandler.h
 * @brief 色領域を検出するクラス
 * @author sadomiya-sousi
 */

#ifndef COLOR_REGION_DETECTION_ACTION_HANDLER_H
#define COLOR_REGION_DETECTION_ACTION_HANDLER_H

#include "SystemInfo.h"
#include "CameraCapture.h"
#include "SocketProtocol.h"
#include "ColorRegionDetector.h"

#include <functional>
#include "MultiThread.h"
#include "FrameSave.h"

#include <opencv2/core.hpp>

class ColorRegionDetectionActionHandler {
 public:
  /**
   * @brief コンストラクタ
   * @param camera カメラキャプチャのインスタンス
   */
  explicit ColorRegionDetectionActionHandler(CameraCapture& camera);

  /**
   * @brief デストラクタ
   */
  ~ColorRegionDetectionActionHandler();

  /**
   * @brief 色領域検出アクションを実行する
   * @param request クライアントからのリクエスト
   * @param response クライアントへのレスポンス
   */
  void execute(const CameraServer::ColorRegionDetectorRequest& request,
               CameraServer::ColorRegionDetectorResponse& response);

 private:
  CameraCapture& camera;
  ColorRegionDetector detector;

  // 画像座標からワールド座標への射影変換行列
  cv::Mat homography;

  /**
   * @brief ホモグラフィ行列を初期化する
   */
  void initializeHomography();

  /**
   * @brief 画像座標をワールド座標（mm）に変換する
   * @param pixelX 画像上のX座標
   * @param pixelY 画像上のY座標
   * @return ワールド座標（変換失敗時はNaN）
   */
  cv::Point2f pixelToWorld(double pixelX, double pixelY) const;
};

#endif  // COLOR_REGION_DETECTION_ACTION_HANDLER_H