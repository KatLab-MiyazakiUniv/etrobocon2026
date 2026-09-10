/**
 * @file ColorRegionDetectionActionHandler.cpp
 * @brief 色領域を検出するクラス
 * @author sadomiya-sousi
 */

#include "ColorRegionDetectionActionHandler.h"

ColorRegionDetectionActionHandler::ColorRegionDetectionActionHandler(CameraCapture& _camera)
  : camera(_camera),
    detector({ { cv::Scalar(0, 0, 0, 0), cv::Scalar(180, 255, 30, 0) } },
             cv::Rect(0, 0, CAM_MAX_WIDTH, CAM_MAX_HEIGHT)),
    qrDetector(cv::Rect(0, 0, CAM_MAX_WIDTH, CAM_MAX_HEIGHT))
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
  int totalStartTime = ClockUtil::now();

  // 1. フレーム取得の計測
  int t1 = ClockUtil::now();
  cv::Mat frame;
  if(!camera.getFrame(frame)) {
    Logger::error("ColorRegionDetectionActionHandler:フレームの取得に失敗しました");
    response.result.wasDetected = false;
    return;
  }
  int t2 = ClockUtil::now();
  Logger::printfLog(Logger::INFO, "[Perf] フレーム取得時間: %d ms", t2 - t1);

  // 2. HSVレンジの設定処理の計測
  int t_hsv_start = ClockUtil::now();
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
  detector.setValidatedRoi(localRoi);
  int t_hsv_end = ClockUtil::now();
  Logger::printfLog(Logger::INFO, "[Perf] HSVレンジ設定・準備時間: %d ms", t_hsv_end - t_hsv_start);

  // 3. 色領域検出の計測
  int t_detect_start = ClockUtil::now();
  BoundingBoxDetectionResult localResult;
  if(request.requireLargestColorIndex) {
    detector.detect(frame, localResult, response.largestColorIndex);
  } else {
    detector.detect(frame, localResult);
  }
  int t_detect_end = ClockUtil::now();
  Logger::printfLog(Logger::INFO, "[Perf] 色領域検出(detector.detect)時間: %d ms",
                    t_detect_end - t_detect_start);

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

  // 4. QRコード検出（非同期処理内）の計測
  int t_qr_dispatch_start = ClockUtil::now();
  MultiThread::wrap([=]() mutable {
    int t_inner_qr_start = ClockUtil::now();
    QrCodeDetectionResult qrResult = qrDetector.detect(frame);
    int t_inner_qr_end = ClockUtil::now();
    Logger::printfLog(Logger::INFO, "[Perf] 【別スレッド内】QRコード検出処理時間: %d ms",
                      t_inner_qr_end - t_inner_qr_start);

    if(qrResult.wasDetected) {
      Logger::printfLog(Logger::INFO,
                        "ColorRegionDetectionActionHandler:QRコードの検出に成功しました: %s",
                        qrResult.content.c_str());
    }
  });
  int t_qr_dispatch_end = ClockUtil::now();
  Logger::printfLog(Logger::INFO, "[Perf] QRコード非同期タスク発行時間: %d ms",
                    t_qr_dispatch_end - t_qr_dispatch_start);

  // 5. フレーム保存（非同期処理内）の計測
  std::string directoryPath = "datafiles/line_trace";
  // int t_save_dispatch_start = ClockUtil::now();
  // MultiThread::wrap([=]() mutable {
  //   int t_inner_save_start = ClockUtil::now();
  //   FrameSave::save(frame, directoryPath, localResult, localRoi);
  //   int t_inner_save_end = ClockUtil::now();
  //   Logger::printfLog(Logger::INFO, "[Perf] 【別スレッド内】フレーム保存処理時間: %d ms",
  //                     t_inner_save_end - t_inner_save_start);
  // });
  // int t_save_dispatch_end = ClockUtil::now();
  // Logger::printfLog(Logger::INFO, "[Perf] フレーム保存非同期タスク発行時間: %d ms",
  //                   t_save_dispatch_end - t_save_dispatch_start);

  int totalEndTime = ClockUtil::now();
  Logger::printfLog(Logger::INFO, "[Perf] execute全体処理時間: %d ms",
                    totalEndTime - totalStartTime);
}
