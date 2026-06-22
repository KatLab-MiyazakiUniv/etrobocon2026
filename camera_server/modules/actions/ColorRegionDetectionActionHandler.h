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
#include "FrameSave.h"
#include "ClockUtil.h"



class ColorRegionDetectionActionHandler {
 public:
  /**
   * @brief コンストラクタ
   * @param camera カメラキャプチャのインスタンス
   */
  ColorRegionDetectionActionHandler(CameraCapture& camera);

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

  /**
   * @brief カメラキャプチャのインスタンスを取得する
   * @return CameraCapture& カメラキャプチャのインスタンスへの参照
   */
  const CameraCapture& getCamera() const;

  /**
   * @brief 色領域検出器を取得する
   * @return const ColorRegionDetector& 色領域検出器への参照
   */
  const ColorRegionDetector& getDetector() const;

 private:
  CameraCapture& camera;
  ColorRegionDetector detector;
};

#endif  // COLOR_REGION_DETECTION_ACTION_HANDLER_H