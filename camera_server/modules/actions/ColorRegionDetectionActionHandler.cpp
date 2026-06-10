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
  ColorRegionDetector detector(localHsvRanges, localRoi);
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
    std::cout << "Color region not detected." << std::endl;
    Logger::error("Color region not detected");
  }
}

// /**
//  * @file ColorRegionDetectionActionHandler.cpp
//  * @brief 色領域をするクラス
//  * @author sadomiya-sousi
//  */

// #include "ColorRegionDetectionActionHandler.h"

// ColorRegionDetectionActionHandler::ColorRegionDetectionActionHandler(CameraCapture& _camera)
//   : camera(_camera)
// {
// }

// void ColorRegionDetectionActionHandler::execute(
//     const CameraServer::ColorRegionDetectorRequest& request,
//     CameraServer::ColorRegionDetectorResponse& response)
// {
//   cv::Mat frame;
//   if(!camera.getFrame(frame)) {
//     response.result.wasDetected = false;
//     return;
//   }

//   // hsvRanges の型変換 (CameraServer::HSVRangeData[] から std::vector<HSVRange> へ)
//   // 動的配列だから事前に要素数を決めておく必要がないのか
//   std::vector<ColorRegionDetector::HSVRange> temp_hsvRanges;
//   temp_hsvRanges.reserve(request.hsvRangeCount);
//   for(int i = 0; i < request.hsvRangeCount; i++) {
//     ColorRegionDetector::HSVRange range;
//     range.lower = cv::Scalar(request.hsvRanges[i].lower.v0, request.hsvRanges[i].lower.v1,
//                              request.hsvRanges[i].lower.v2, request.hsvRanges[i].lower.v3);
//     range.upper = cv::Scalar(request.hsvRanges[i].upper.v0, request.hsvRanges[i].upper.v1,
//                              request.hsvRanges[i].upper.v2, request.hsvRanges[i].upper.v3);
//     // なんだコレ>temp_hsvRanges>vector<HSVRange>型,
//     //
//     rangはHSVRangeやな,vector<HSVRange>の1要素分を用意して代入してるのか..>動的な配列であるvector<>は,
//     //
//     配列の末尾に要素を追加するためにpush_back()を使用する..>これにより動的に末尾のメモリを確保できる
//     temp_hsvRanges.push_back(range);
//   }

//   // roi の型変換 (CameraServer::RectData から cv::Rect へ)
//   cv::Rect temp_roi(request.roi.x, request.roi.y, request.roi.width, request.roi.height);

//   // 検出器の初期化
//   ColorRegionDetector detector(temp_hsvRanges, temp_roi);

//   // 検出結果の一時格納用 (ImageRecognitionResults.h の BoundingBoxDetectionResult)
//   BoundingBoxDetectionResult local_result;

//   // requireLargestColorIndex に応じて適切な detect を呼び出し、結果を格納する
//   if(request.requireLargestColorIndex) {
//     detector.detect(frame, local_result, response.largestColorIndex);
//   } else {
//     detector.detect(frame, local_result);
//   }

//   // 検出結果をレスポンス用構造体にコピー・変換する
//   // response.result.wasDetected = local_result.wasDetected;
//   if(respones.result.wasDetected) {
//     response.result.topLeft.x = local_result.topLeft.x;
//     response.result.topLeft.y = local_result.topLeft.y;
//     response.result.topRight.x = local_result.topRight.x;
//     response.result.topRight.y = local_result.topRight.y;
//     response.result.bottomLeft.x = local_result.bottomLeft.x;
//     response.result.bottomLeft.y = local_result.bottomLeft.y;
//     response.result.bottomRight.x = local_result.bottomRight.x;
//     response.result.bottomRight.y = local_result.bottomRight.y;
//     Logger::info("Color region detected successfully");
//   } else {
//     std::cout << "Color region not detected." << std::endl;
//     Logger::error("Color region not detected")
//   }
// }

// // /**
// //  * @file ColorRegionDetectionActionHandler.cpp
// //  * @brief 色領域をするクラス
// //  * @author sadomiya-sousi
// //  */

// // #include "ColorRegionDetectionActionHandler.h"

// // ColorRegionDetectionActionHandler::ColorRegionDetectionActionHandler(CameraCapture& _camera)
// //   : camera(_camera)
// // {
// // }

// // void ColorRegionDetectionActionHandler::execute(
// //     const CameraServer::ColorRegionDetectorRequest& request,
// //     CameraServer::ColorRegionDetectorResponse& response)
// // {
// //   //   std::cout << "Executing TWO_COLOR_LINE_DETECTION command." << std::endl;

// //   cv::Mat frame;
// //   if(!camera.getFrame(frame)) {
// //     // std::cerr << "Failed to capture frame for line detection." << std::endl;
// //     response.result.wasDetected = false;
// //     return;
// //   }

// //   cv::Rect temp_roi = { request.roi.x, request.roi.y, request.roi.width, request.roi.height };
// //   ColorRegionDetector::HSVRange temp_hsvRange;
// //   for(int i = 0; i < request.hsvRangeCount; i++) {
// //     //どうやって型変換をすればいいの?
// //     request.hsvRanges
// //   }
// //   cv::Rect temp_roi = { request.roi.x, request.roi.y, request.roi.width, request.roi.height };
// //   // 型変換が必要何だけど文法がわからない。
// //   // ColorRegionDetector detector(temp_request.hsvRanges, temp_request.roi);
// //   if(request.requireLargestColorIndex) {
// //     detector.detect(frame, response.result);
// //   }

// //   else {
// //     // 最も領域の大きい色領域を返すようのdetect
// //     detector.detect(frame, response.result, request.largestColorIndex);
// //   }
// //   if(response.result.wasDetected) {
// //     std::cout << "Line detected successfully." << std::endl;
// //   }

// //   else {
// //     std::cout << "Line not detected." << std::endl;
// //   }
// // }
