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
  LOG_CREATE("ColorRegionDetectionActionHandler");
}

ColorRegionDetectionActionHandler::~ColorRegionDetectionActionHandler()
{
  LOG_DESTROY("ColorRegionDetectionActionHandler");
}

void ColorRegionDetectionActionHandler::execute(
    const CameraServer::ColorRegionDetectorRequest& request,
    CameraServer::ColorRegionDetectorResponse& response)
{
  int startTime = ClockUtil::now();

  //---------------------------------
  // Frame Acquire
  //---------------------------------
  int t1 = ClockUtil::now();

  cv::Mat frame;
  if(!camera.getFrame(frame)) {
    Logger::error("ColorRegionDetectionActionHandler:フレームの取得に失敗しました");
    response.result.wasDetected = false;
    return;
  }

  int t2 = ClockUtil::now();

  Logger::printfLog(Logger::INFO, "FrameAcquire : %d ms", t2 - t1);

  //---------------------------------
  // HSV Range Convert
  //---------------------------------
  t1 = ClockUtil::now();

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

  int t3 = ClockUtil::now();

  Logger::printfLog(Logger::INFO, "HSVRangeConvert : %d ms", t3 - t1);

  //---------------------------------
  // ROI Create
  //---------------------------------
  t1 = ClockUtil::now();

  cv::Rect localRoi(request.roi.x, request.roi.y, request.roi.width, request.roi.height);

  int t4 = ClockUtil::now();

  Logger::printfLog(Logger::INFO, "RoiCreate : %d ms", t4 - t1);

  //---------------------------------
  // Detector Setup
  //---------------------------------
  t1 = ClockUtil::now();

  detector.setHsvRanges(localHsvRanges);
  detector.setRoi(localRoi);

  int t5 = ClockUtil::now();

  Logger::printfLog(Logger::INFO, "DetectorSetup : %d ms", t5 - t1);

  //---------------------------------
  // Detect
  //---------------------------------
  BoundingBoxDetectionResult localResult;

  t1 = ClockUtil::now();

  if(request.requireLargestColorIndex) {
    detector.detect(frame, localResult, response.largestColorIndex);
  } else {
    detector.detect(frame, localResult);
  }

  int t6 = ClockUtil::now();

  Logger::printfLog(Logger::INFO, "Detect : %d ms", t6 - t1);

  //---------------------------------
  // Response Copy
  //---------------------------------
  t1 = ClockUtil::now();

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
  }

  int t7 = ClockUtil::now();

  Logger::printfLog(Logger::INFO, "ResponseCopy : %d ms", t7 - t1);

  //---------------------------------
  // Frame Save
  //---------------------------------
  t1 = ClockUtil::now();

  // std::string directoryPath = "datafiles/line_trace";
  std::string directoryPath = "datafiles/line_trace";

  FrameSave::save(frame, directoryPath, localResult, localRoi);

  int t8 = ClockUtil::now();

  Logger::printfLog(Logger::INFO, "FrameSave : %d ms", t8 - t1);

  //---------------------------------
  // Total
  //---------------------------------
  int endTime = ClockUtil::now();

  Logger::printfLog(Logger::INFO, "ColorRegionDetectionActionHandler TOTAL : %d ms",
                    endTime - startTime);
}

// void ColorRegionDetectionActionHandler::execute(
//     const CameraServer::ColorRegionDetectorRequest& request,
//     CameraServer::ColorRegionDetectorResponse& response)
// {
//   int beforeFrameSaveTime = ClockUtil::now();
//   cv::Mat frame;
//   if(!camera.getFrame(frame)) {
//     Logger::error("ColorRegionDetectionActionHandler:フレームの取得に失敗しました");
//     response.result.wasDetected = false;
//     return;
//   }
//   int afterFrameSaveTime = ClockUtil::now();
//   Logger::printfLog(Logger::INFO, "ColorRegionAcitionHandler: フレーム取得にかかった時間は%d",
//                     afterFrameSaveTime - beforeFrameSaveTime);

//   std::vector<ColorRegionDetector::HSVRange> localHsvRanges;
//   localHsvRanges.reserve(request.hsvRangeCount);
//   for(int i = 0; i < request.hsvRangeCount; i++) {
//     ColorRegionDetector::HSVRange range;
//     range.lower = cv::Scalar(request.hsvRanges[i].lower.h, request.hsvRanges[i].lower.s,
//                              request.hsvRanges[i].lower.v);
//     range.upper = cv::Scalar(request.hsvRanges[i].upper.h, request.hsvRanges[i].upper.s,
//                              request.hsvRanges[i].upper.v);
//     localHsvRanges.push_back(range);
//   }

//   cv::Rect localRoi(request.roi.x, request.roi.y, request.roi.width, request.roi.height);

//   detector.setHsvRanges(localHsvRanges);
//   detector.setRoi(localRoi);
//   BoundingBoxDetectionResult localResult;

//   if(request.requireLargestColorIndex) {
//     detector.detect(frame, localResult, response.largestColorIndex);
//   } else {
//     detector.detect(frame, localResult);
//   }

//   response.result.wasDetected = localResult.wasDetected;
//   if(localResult.wasDetected) {
//     response.result.topLeft.x = localResult.topLeft.x;
//     response.result.topLeft.y = localResult.topLeft.y;
//     response.result.topRight.x = localResult.topRight.x;
//     response.result.topRight.y = localResult.topRight.y;
//     response.result.bottomLeft.x = localResult.bottomLeft.x;
//     response.result.bottomLeft.y = localResult.bottomLeft.y;
//     response.result.bottomRight.x = localResult.bottomRight.x;
//     response.result.bottomRight.y = localResult.bottomRight.y;

//     Logger::info("ColorRegionDetectionActionHandler:色領域の検出に成功しました");

//   } else {
//     Logger::error("ColorRegionDetectionActionHandler:色領域が検出されませんでした");
//   }

//   beforeFrameSaveTime = ClockUtil::now();
//   // フレームを保存する（動画作成ツール用）
//   std::string directoryPath = "datafiles/line_trace";
//   FrameSave::save(frame, directoryPath, localResult);
//   Logger::printfLog(Logger::INFO, "ColorRegionAcitionHandler: フレーム保存にかかった時間は%dms",
//                     afterFrameSaveTime - beforeFrameSaveTime);
//   afterFrameSaveTime = ClockUtil::now();
// }

const CameraCapture& ColorRegionDetectionActionHandler::getCamera() const
{
  return camera;
}

const ColorRegionDetector& ColorRegionDetectionActionHandler::getDetector() const
{
  return detector;
}