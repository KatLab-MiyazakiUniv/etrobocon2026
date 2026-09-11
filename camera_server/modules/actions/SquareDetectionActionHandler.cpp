/**
 * @file   SquareDetectionActionHandler.cpp
 * @brief  正方形検出要求を処理するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "SquareDetectionActionHandler.h"

SquareDetectionActionHandler::SquareDetectionActionHandler(
    CameraCapture& _camera)
  : camera(_camera),
    detector(
        cv::Rect(
            0,
            0,
            CAM_MAX_WIDTH,
            CAM_MAX_HEIGHT))
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

  cv::Mat frame;

  if(!camera.getFrame(frame)) {

    Logger::error(
        "SquareDetectionActionHandler: "
        "フレームの取得に失敗しました");

    response.wasDetected = false;

    return;
  }

  const cv::Rect localRoi(
      request.roi.x,
      request.roi.y,
      request.roi.width,
      request.roi.height);

  detector.setValidatedRoi(
      localRoi);

  BoundingBoxDetectionResult result {};

  detector.detect(
      frame,
      result);

  if(!result.wasDetected) {

    Logger::warning(
        "SquareDetectionActionHandler: "
        "正方形を検出できませんでした");

    response.wasDetected = false;

    return;
  }

  response.wasDetected = true;

  response.corners[0].x
      = result.topLeft.x;

  response.corners[0].y
      = result.topLeft.y;

  response.corners[1].x
      = result.topRight.x;

  response.corners[1].y
      = result.topRight.y;

  response.corners[2].x
      = result.bottomRight.x;

  response.corners[2].y
      = result.bottomRight.y;

  response.corners[3].x
      = result.bottomLeft.x;

  response.corners[3].y
      = result.bottomLeft.y;

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "square detected "
      "TL=(%d,%d) "
      "TR=(%d,%d) "
      "BR=(%d,%d) "
      "BL=(%d,%d)",
      result.topLeft.x,
      result.topLeft.y,
      result.topRight.x,
      result.topRight.y,
      result.bottomRight.x,
      result.bottomRight.y,
      result.bottomLeft.x,
      result.bottomLeft.y);
}