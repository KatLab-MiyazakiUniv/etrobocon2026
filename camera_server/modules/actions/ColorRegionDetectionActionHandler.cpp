/**
 * @file ColorRegionDetectionActionHandler.cpp
 * @brief 色領域を検出するクラス
 * @author sadomiya-sousi
 */

#include "ColorRegionDetectionActionHandler.h"

ColorRegionDetectionActionHandler::ColorRegionDetectionActionHandler(CameraCapture& _camera)
  : camera(_camera),
    detector({ { cv::Scalar(0, 0, 0, 0), cv::Scalar(180, 255, 30, 0) } },
             cv::Rect(0, 0, 1920, 1080))
{
}

void ColorRegionDetectionActionHandler::execute(
    const CameraServer::ColorRegionDetectorRequest& request,
    CameraServer::ColorRegionDetectorResponse& response)
{
  cv::Mat frame;
  if(!camera.getFrame(frame)) {
    Logger::error("filed to get frame");
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

    Logger::info("Color region detected successfully");

  } else {
    Logger::error("Color region not detected");
  }
}