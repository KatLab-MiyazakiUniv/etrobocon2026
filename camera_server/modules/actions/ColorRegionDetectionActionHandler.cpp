/**
 * @file ColorRegionDetectionActionHandler.cpp
 * @brief 色領域を検出するクラス
 * @author sadomiya-sousi
 */

#include "ColorRegionDetectionActionHandler.h"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace {

  // 正方形検出で使用している校正値
  constexpr double BOARD_WIDTH = 200.0;
  constexpr double CAMERA_TO_BOARD_NEAR_EDGE = 200.0;

  // 画像座標とワールド座標の対応
  const std::vector<cv::Point2f> IMAGE_POINTS = {
    { 729.13f, 408.15f },  { 963.67f, 408.06f }, { 1198.66f, 407.19f }, { 705.05f, 537.40f },
    { 1228.23f, 536.45f }, { 674.65f, 699.43f }, { 970.28f, 698.92f },  { 1265.69f, 698.89f },
  };

  const std::vector<cv::Point2f> WORLD_POINTS = {
    { 50.0f, 150.0f },  { 100.0f, 150.0f }, { 150.0f, 150.0f }, { 50.0f, 100.0f },
    { 150.0f, 100.0f }, { 50.0f, 50.0f },   { 100.0f, 50.0f },  { 150.0f, 50.0f },
  };

}  // namespace

ColorRegionDetectionActionHandler::ColorRegionDetectionActionHandler(CameraCapture& _camera)
  : camera(_camera),
    detector({ { cv::Scalar(0, 0, 0, 0), cv::Scalar(180, 255, 30, 0) } },
             cv::Rect(0, 0, CAM_MAX_WIDTH, CAM_MAX_HEIGHT))
{
  LOG_CREATE("ColorRegionDetectionActionHandler");

  initializeHomography();
}

ColorRegionDetectionActionHandler::~ColorRegionDetectionActionHandler()
{
  LOG_DESTROY("ColorRegionDetectionActionHandler");
}

void ColorRegionDetectionActionHandler::initializeHomography()
{
  homography = cv::findHomography(IMAGE_POINTS, WORLD_POINTS, 0);

  if(homography.empty()) {
    Logger::error("ColorRegionDetectionActionHandler:ホモグラフィ行列の初期化に失敗しました");
    return;
  }

  homography.convertTo(homography, CV_64F);

  Logger::info("ColorRegionDetectionActionHandler:ホモグラフィ行列を初期化しました");
}

cv::Point2f ColorRegionDetectionActionHandler::pixelToWorld(double pixelX, double pixelY) const
{
  const float nan = std::numeric_limits<float>::quiet_NaN();

  if(homography.empty()) {
    return cv::Point2f(nan, nan);
  }

  std::vector<cv::Point2f> imagePoints = {
    cv::Point2f(static_cast<float>(pixelX), static_cast<float>(pixelY)),
  };

  std::vector<cv::Point2f> worldPoints;

  cv::perspectiveTransform(imagePoints, worldPoints, homography);

  if(worldPoints.empty()) {
    return cv::Point2f(nan, nan);
  }

  return worldPoints.front();
}

void ColorRegionDetectionActionHandler::execute(
    const CameraServer::ColorRegionDetectorRequest& request,
    CameraServer::ColorRegionDetectorResponse& response)
{
  const int totalStartTime = ClockUtil::now();

  // 前回の値が残らないよう、レスポンス全体を初期化
  response = CameraServer::ColorRegionDetectorResponse{};

  if(homography.empty()) {
    Logger::error("ColorRegionDetectionActionHandler:ホモグラフィ行列がありません");
    return;
  }

  cv::Mat frame;

  if(!camera.getFrame(frame)) {
    Logger::error("ColorRegionDetectionActionHandler:フレームの取得に失敗しました");
    return;
  }

  // リクエストのHSV範囲を設定
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

  const cv::Rect localRoi(request.roi.x, request.roi.y, request.roi.width, request.roi.height);

  detector.setHsvRanges(localHsvRanges);
  detector.setValidatedRoi(localRoi);

  // 色領域を検出
  BoundingBoxDetectionResult localResult;

  if(request.requireLargestColorIndex) {
    detector.detect(frame, localResult, response.largestColorIndex);
  } else {
    detector.detect(frame, localResult);
  }

  response.result.wasDetected = localResult.wasDetected;

  if(localResult.wasDetected) {
    // バウンディングボックスの四隅をレスポンスに格納
    response.result.topLeft.x = localResult.topLeft.x;
    response.result.topLeft.y = localResult.topLeft.y;

    response.result.topRight.x = localResult.topRight.x;
    response.result.topRight.y = localResult.topRight.y;

    response.result.bottomLeft.x = localResult.bottomLeft.x;
    response.result.bottomLeft.y = localResult.bottomLeft.y;

    response.result.bottomRight.x = localResult.bottomRight.x;
    response.result.bottomRight.y = localResult.bottomRight.y;

    // バウンディングボックスの中心（画像座標）
    const double centerX = (localResult.topLeft.x + localResult.topRight.x) / 2.0;

    const double centerY = (localResult.topLeft.y + localResult.bottomLeft.y) / 2.0;

    response.centerX = centerX;
    response.centerY = centerY;

    // 画像座標からワールド座標（mm）へ変換
    const cv::Point2f worldPoint = pixelToWorld(centerX, centerY);

    if(!std::isfinite(worldPoint.x) || !std::isfinite(worldPoint.y)) {
      // 検出自体は成功しているが、距離計算は失敗
      response.forwardDistance = 0.0;
      response.lateralDistance = 0.0;

      Logger::error("ColorRegionDetectionActionHandler:座標変換に失敗しました");
    } else {
      // 正方形検出と同じ座標系・距離基準を使用
      response.lateralDistance = static_cast<double>(worldPoint.x) - BOARD_WIDTH / 2.0;

      response.forwardDistance = CAMERA_TO_BOARD_NEAR_EDGE + static_cast<double>(worldPoint.y);

      Logger::printfLog(Logger::INFO,
                        "[ColorRegion] center=(%.1f, %.1f) px, "
                        "world=(%.1f, %.1f) mm, "
                        "forward=%.1f mm, lateral=%.1f mm",
                        centerX, centerY, static_cast<double>(worldPoint.x),
                        static_cast<double>(worldPoint.y), response.forwardDistance,
                        response.lateralDistance);
    }

    Logger::info("ColorRegionDetectionActionHandler:色領域の検出に成功しました");
  } else {
    Logger::error("ColorRegionDetectionActionHandler:色領域が検出されませんでした");
  }

  // 既存の画像保存処理を維持
  const std::string directoryPath = "datafiles/line_trace";

  MultiThread::wrap(
      [=]() mutable { FrameSave::save(frame, directoryPath, localResult, localRoi); });

  const int totalEndTime = ClockUtil::now();

  Logger::printfLog(Logger::INFO, "[Perf] execute全体処理時間: %d ms",
                    totalEndTime - totalStartTime);
}