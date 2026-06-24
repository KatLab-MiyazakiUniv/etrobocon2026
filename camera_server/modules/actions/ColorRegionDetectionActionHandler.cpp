/**
 * @file ColorRegionDetectionActionHandler.cpp
 * @brief 色領域を検出するクラス
 * @author sadomiya-sousi
 */

#include "ColorRegionDetectionActionHandler.h"

ColorRegionDetectionActionHandler::ColorRegionDetectionActionHandler()
  : camera(),
    detector({ { cv::Scalar(0, 0, 0, 0), cv::Scalar(180, 255, 30, 0) } },
             cv::Rect(0, 0, 1920, 1080))
{
  LOG_CREATE("ColorRegionDetectionActionHandler");

  // CameraCapture camera;
  int cameraId = camera.findAvailableCameraID();
  if(cameraId < 0) {
    Logger::error("利用可能なカメラを認識失敗");
    return;
  }
  camera.setCameraID(cameraId);
  if(!camera.openCamera()) {
    Logger::error("カメラの起動に失敗");
    return;
  }

  cv::Mat frame;
  if(!camera.getFrame(frame)) {
    Logger::error("ColorRegionDetectionActionHandler:フレームの取得に失敗しました");
    return;
  }
}

ColorRegionDetectionActionHandler::~ColorRegionDetectionActionHandler()
{
  LOG_DESTROY("ColorRegionDetectionActionHandler");
}

void ColorRegionDetectionActionHandler::execute(
    const CameraServer::ColorRegionDetectorRequest& request,
    CameraServer::ColorRegionDetectorResponse& response)
{
  cv::Mat frame;
  if(!camera.getFrame(frame)) {
    Logger::error("ColorRegionDetectionActionHandler:フレームの取得に失敗しました");
    response.result.wasDetected = false;
    return;
  }

  std::vector<ColorRegionDetector::HSVRange> localHsvRanges;
  localHsvRanges.reserve(request.hsvRangeCount);
  for(int i = 0; i < request.hsvRangeCount; i++) {
    ColorRegionDetector::HSVRange range;
    range.lower = cv::Scalar(request.hsvRanges[i].lower.h, request.hsvRanges[i].lower.s,
                             request.hsvRanges[i].lower.v);
    range.upper = cv::Scalar(request.hsvRanges[i].upper.h, request.hsvRanges[i].upper.s,
                             request.hsvRanges[i].upper.v);
    localHsvRanges.push_back(range);
  }

  cv::Rect localRoi(request.roi.x, request.roi.y, request.roi.width, request.roi.height);

  detector.setHsvRanges(localHsvRanges);
  detector.setRoi(localRoi);
  BoundingBoxDetectionResult localResult;

  if(request.requireLargestColorIndex) {
    detector.detect(frame, localResult, response.largestColorIndex);
  } else {
    detector.detect(frame, localResult);
  }

  response.result.wasDetected = localResult.wasDetected;
  if(localResult.wasDetected) {
    response.result.topLeft.x = localResult.topLeft.x;
    response.result.topLeft.y = localResult.topLeft.y;
    response.result.topRight.x = localResult.topRight.x;
    response.result.topRight.y = localResult.topRight.y;
    response.result.bottomLeft.x = localResult.bottomLeft.x;
    response.result.bottomLeft.y = localResult.bottomLeft.y;
    response.result.bottomRight.x = localResult.bottomRight.x;
    response.result.bottomRight.y = localResult.bottomRight.y;

    Logger::info("ColorRegionDetectionActionHandler:色領域の検出に成功しました");

  } else {
    Logger::error("ColorRegionDetectionActionHandler:色領域が検出されませんでした");
  }

  // フレームを保存する（動画作成ツール用）
  std::string directoryPath = "datafiles/line_trace";
  FrameSave::save(frame, directoryPath, localResult);
}

const CameraCapture& ColorRegionDetectionActionHandler::getCamera() const
{
  return camera;
}

const ColorRegionDetector& ColorRegionDetectionActionHandler::getDetector() const
{
  return detector;
}