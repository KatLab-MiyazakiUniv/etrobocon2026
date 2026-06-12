/**
 * @file ColorRegionDetectionActionHandler.cpp
 * @brief 色領域をするクラス
 * @author sadomiya-sousi
 */

#include "ColorRegionDetectionActionHandler.h"

ColorRegionDetectionActionHandler::ColorRegionDetectionActionHandler(CameraCapture& _camera)
  : camera(_camera)
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

  // requestのメンバ変数でColorRegionDetecttorインスタンスで使用する引数を比較し,同じならインスタンス化をスキップする
  bool changedRequest
      = !detector || !hasCachedRequest || cachedRequest.roi.x != request.roi.x
        || cachedRequest.roi.y != request.roi.y || cachedRequest.roi.width != request.roi.width
        || cachedRequest.roi.height != request.roi.height
        || cachedRequest.hsvRangeCount != request.hsvRangeCount
        // ColorRegionDetector::HSVRangeのサイズが8バイトの倍数じゃなくて,パディングが起こる構造体なら,要変更。
        || std::memcmp(cachedRequest.hsvRanges, request.hsvRanges,
                       request.hsvRangeCount * sizeof(CameraServer::HSVRangeData))
               != 0;

  if(changedRequest) {
    std::vector<ColorRegionDetector::HSVRange> localHsvRanges;
    localHsvRanges.reserve(request.hsvRangeCount);
    for(int i = 0; i < request.hsvRangeCount; i++) {
      ColorRegionDetector::HSVRange range;
      range.lower = cv::Scalar(request.hsvRanges[i].lower.v0, request.hsvRanges[i].lower.v1,
                               request.hsvRanges[i].lower.v2, request.hsvRanges[i].lower.v3);
      range.upper = cv::Scalar(request.hsvRanges[i].upper.v0, request.hsvRanges[i].upper.v1,
                               request.hsvRanges[i].upper.v2, request.hsvRanges[i].upper.v3);
      localHsvRanges.push_back(range);
    }

    cv::Rect localRoi(request.roi.x, request.roi.y, request.roi.width, request.roi.height);
    detector = std::make_unique<ColorRegionDetector>(localHsvRanges, localRoi);

    cachedRequest = request;
    hasCachedRequest = true;
  }

  BoundingBoxDetectionResult localResult;

  if(request.requireLargestColorIndex) {
    detector->detect(frame, localResult, response.largestColorIndex);
  } else {
    detector->detect(frame, localResult);
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