/**
 * @file SquareDetectionActionHandler.cpp
 * @brief 正方形検出処理を実行するActionHandler
 */

#include "SquareDetectionActionHandler.h"

#include <memory>

#include "Logger.h"
#include "SystemInfo.h"

SquareDetectionActionHandler::SquareDetectionActionHandler(
    CameraCapture& _camera)
  : camera(_camera),
    detector(
        std::make_unique<SquareDetector>(
            cv::Rect(
                0,
                0,
                CAM_MAX_WIDTH,
                CAM_MAX_HEIGHT)))
{
  LOG_CREATE(
      "SquareDetectionActionHandler");
}

SquareDetectionActionHandler::~SquareDetectionActionHandler()
{
  LOG_DESTROY(
      "SquareDetectionActionHandler");
}

void SquareDetectionActionHandler::execute(
    const CameraServer::SquareDetectorRequest& request,
    CameraServer::SquareDetectorResponse& response)
{
  response = {};

  // =========================================================
  // ROI
  // =========================================================

  const cv::Rect localRoi(
      request.roi.x,
      request.roi.y,
      request.roi.width,
      request.roi.height);

  // =========================================================
  // Trackingリセット
  //
  // Straightを挟んだ後の最初のSquare検出では
  // 新しいSquareDetectorを生成する。
  //
  // これによりpreviousCenterやmissedCountなど
  // SquareDetector内部の追跡状態が完全に初期化される。
  // =========================================================

  if(request.resetTracking) {

    Logger::info(
        "SquareDetectionActionHandler: "
        "===== TRACKING RESET =====");

    detector =
        std::make_unique<SquareDetector>(
            localRoi);
  }

  if(!detector) {

    detector =
        std::make_unique<SquareDetector>(
            localRoi);
  }

  // =========================================================
  // ROI設定
  // =========================================================

  detector->setValidatedRoi(
      localRoi);

  // =========================================================
  // カメラ画像取得
  // =========================================================

  cv::Mat frame;

  if(!camera.getFrame(
         frame)) {

    Logger::error(
        "SquareDetectionActionHandler:"
        "フレームの取得に失敗しました");

    response.wasDetected =
        false;

    return;
  }

  // =========================================================
  // 正方形検出
  // =========================================================

  BoundingBoxDetectionResult result {};

  detector->detect(
      frame,
      result);

  // =========================================================
  // 未検出
  // =========================================================

  if(!result.wasDetected) {

    Logger::error(
        "SquareDetectionActionHandler:"
        "正方形が検出されませんでした");

    response.wasDetected =
        false;

    return;
  }

  // =========================================================
  // 検出成功
  // =========================================================

  response.wasDetected =
      true;

 response.corners[0].x = result.topLeft.x;
response.corners[0].y = result.topLeft.y;

response.corners[1].x = result.topRight.x;
response.corners[1].y = result.topRight.y;

response.corners[2].x = result.bottomRight.x;
response.corners[2].y = result.bottomRight.y;

response.corners[3].x = result.bottomLeft.x;
response.corners[3].y = result.bottomLeft.y;

  Logger::info(
      "SquareDetectionActionHandler:"
      "正方形の検出に成功しました");
}