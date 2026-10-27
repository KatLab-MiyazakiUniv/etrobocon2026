
/**
 * @file   SquareDetectionActionHandler.cpp
 * @brief  正方形検出要求を処理し、検出結果を実距離へ変換するクラス
 *         画像取得・正方形検出・距離と角度算出の処理時間を計測する
 * @author yutaro-1214
 */

#include "SquareDetectionActionHandler.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace {

  /**
   * @brief 画像上の基準点[px]
   * 1920×1080画像から取得した8個の黒点中心座標
   */
  const std::vector<cv::Point2f> IMAGE_POINTS = {
    cv::Point2f(729.13F, 408.15F),   // (50, 150)
    cv::Point2f(963.67F, 408.06F),   // (100, 150)
    cv::Point2f(1198.66F, 407.19F),  // (150, 150)
    cv::Point2f(705.05F, 537.40F),   // (50, 100)
    cv::Point2f(1228.23F, 536.45F),  // (150, 100)
    cv::Point2f(674.65F, 699.43F),   // (50, 50)
    cv::Point2f(970.28F, 698.92F),   // (100, 50)
    cv::Point2f(1265.69F, 698.89F)  // (150, 50)
  };

  /**
   * @brief 校正用紙上の基準点[mm]
   */
  const std::vector<cv::Point2f> WORLD_POINTS = {
    cv::Point2f(50.0F, 150.0F),
    cv::Point2f(100.0F, 150.0F),
    cv::Point2f(150.0F, 150.0F),
    cv::Point2f(50.0F, 100.0F),
    cv::Point2f(150.0F, 100.0F),
    cv::Point2f(50.0F, 50.0F),
    cv::Point2f(100.0F, 50.0F),
    cv::Point2f(150.0F, 50.0F)
  };

  /**
   * @brief 校正用紙の横幅[mm]
   */
  constexpr double BOARD_WIDTH = 200.0;

  /**
   * @brief 校正用紙の奥行き[mm]
   */
  constexpr double BOARD_HEIGHT = 200.0;

  /**
   * @brief カメラから校正用紙手前端までの距離[mm]
   */
  constexpr double CAMERA_TO_BOARD_NEAR_EDGE = 200.0;

  /**
   * @brief 1回の検出で取得するフレーム数
   */
  constexpr int FRAME_CAPTURE_COUNT = 3;

  /**
   * @brief 検出回数
   */
  int detectionCount = 0;

}  // namespace

SquareDetectionActionHandler::SquareDetectionActionHandler(CameraCapture& _camera)
  : camera(_camera), detector(cv::Rect(0, 0, CAM_MAX_WIDTH, CAM_MAX_HEIGHT))
{
  initializeHomography();

  LOG_CREATE("SquareDetectionActionHandler");
}

SquareDetectionActionHandler::~SquareDetectionActionHandler()
{
  LOG_DESTROY("SquareDetectionActionHandler");
}

void SquareDetectionActionHandler::initializeHomography()
{
  if(IMAGE_POINTS.size() != WORLD_POINTS.size() || IMAGE_POINTS.size() < 4) {
    Logger::error("SquareDetectionActionHandler:"
                  "ホモグラフィ計算用の基準点が不正です");
    homography.release();
    return;
  }

  // 8点の基準点からホモグラフィ行列を計算
  homography = cv::findHomography(IMAGE_POINTS, WORLD_POINTS, 0);

  if(homography.empty()) {
    Logger::error("SquareDetectionActionHandler:"
                  "ホモグラフィ行列の計算に失敗しました");
    return;
  }

  homography.convertTo(homography, CV_64F);

  Logger::printfLog(Logger::INFO,
                    "SquareDetectionActionHandler: "
                    "ホモグラフィ行列を初期化しました");
}

cv::Point2f SquareDetectionActionHandler::pixelToWorld(double pixelX, double pixelY) const
{
  if(homography.empty()) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    return cv::Point2f(nan, nan);
  }

  const std::vector<cv::Point2f> imagePoints
      = { cv::Point2f(static_cast<float>(pixelX), static_cast<float>(pixelY)) };

  std::vector<cv::Point2f> worldPoints;

  cv::perspectiveTransform(imagePoints, worldPoints, homography);

  if(worldPoints.empty()) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    return cv::Point2f(nan, nan);
  }

  return worldPoints[0];
}

void SquareDetectionActionHandler::execute(const CameraServer::SquareDetectorRequest& request,
                                           CameraServer::SquareDetectorResponse& response)
{
  // =====================================================
  // 距離と角度の算出時間 計測開始
  // =====================================================

  const auto startTime = std::chrono::steady_clock::now();

  response = {};
  ++detectionCount;

  // =====================================================
  // ホモグラフィ確認
  // =====================================================

  if(homography.empty()) {
    Logger::error("SquareDetectionActionHandler:"
                  "ホモグラフィ行列が初期化されていません");
    response.wasDetected = false;
    return;
  }

  // =====================================================
  // 1. カメラ画像取得
  //
  // 古いフレームを避けるため3フレーム取得
  // 最後のフレームのみを検出に使用する
  // =====================================================

  cv::Mat frame;

  for(int i = 0; i < FRAME_CAPTURE_COUNT; ++i) {
    cv::Mat currentFrame;

    if(!camera.getFrame(currentFrame)) {
      Logger::error("SquareDetectionActionHandler:"
                    "フレームの取得に失敗しました");
      response.wasDetected = false;
      return;
    }

    // 検出には最後のフレームしか使用しないため、コピーも最後だけにする
    if(i == FRAME_CAPTURE_COUNT - 1) {
      frame = currentFrame.clone();
    }
  }

  // 画像取得終了時刻
  const auto captureEndTime = std::chrono::steady_clock::now();

  // =====================================================
  // 2. ROI設定
  // =====================================================

  const cv::Rect roi(request.roi.x, request.roi.y, request.roi.width, request.roi.height);

  detector.setValidatedRoi(roi);

  // =====================================================
  // 3. 正方形検出
  // =====================================================

  BoundingBoxDetectionResult detectionResult{};

  detector.detect(frame, detectionResult);

  // 正方形検出終了時刻（ROI設定の時間を含む）
  const auto detectEndTime = std::chrono::steady_clock::now();

  if(!detectionResult.wasDetected) {
    Logger::printfLog(Logger::WARNING,
                      "SquareDetectionActionHandler: "
                      "検出 #%d 正方形を検出できませんでした",
                      detectionCount);

    response.wasDetected = false;

    const double captureMs
        = std::chrono::duration<double, std::milli>(captureEndTime - startTime).count();
    const double detectMs
        = std::chrono::duration<double, std::milli>(detectEndTime - captureEndTime).count();
    Logger::printfLog(Logger::INFO,
                      "[SquareProfile] status=NOT_DETECTED capture=%.2f ms "
                      "detect=%.2f ms total=%.2f ms",
                      captureMs, detectMs, captureMs + detectMs);
    return;
  }

  // =====================================================
  // 4. 画像上の正方形中心座標
  // =====================================================

  const double centerX = (detectionResult.topLeft.x + detectionResult.topRight.x
                          + detectionResult.bottomRight.x + detectionResult.bottomLeft.x)
                         / 4.0;

  const double centerY = (detectionResult.topLeft.y + detectionResult.topRight.y
                          + detectionResult.bottomRight.y + detectionResult.bottomLeft.y)
                         / 4.0;

  // =====================================================
  // 5. 4頂点を実座標へ変換
  // =====================================================

  const std::vector<cv::Point2f> imageCorners
      = { cv::Point2f(static_cast<float>(detectionResult.topLeft.x),
                      static_cast<float>(detectionResult.topLeft.y)),
          cv::Point2f(static_cast<float>(detectionResult.topRight.x),
                      static_cast<float>(detectionResult.topRight.y)),
          cv::Point2f(static_cast<float>(detectionResult.bottomRight.x),
                      static_cast<float>(detectionResult.bottomRight.y)),
          cv::Point2f(static_cast<float>(detectionResult.bottomLeft.x),
                      static_cast<float>(detectionResult.bottomLeft.y)) };

  std::vector<cv::Point2f> worldCorners;

  cv::perspectiveTransform(imageCorners, worldCorners, homography);

  if(worldCorners.size() != imageCorners.size()) {
    Logger::warning("SquareDetectionActionHandler:"
                    "ホモグラフィ変換に失敗しました");
    response.wasDetected = false;
    return;
  }

  // =====================================================
  // 6. 実座標の中心を計算
  // =====================================================

  double worldX = 0.0;
  double worldY = 0.0;

  for(const cv::Point2f& point : worldCorners) {
    worldX += static_cast<double>(point.x);
    worldY += static_cast<double>(point.y);
  }

  worldX /= static_cast<double>(worldCorners.size());
  worldY /= static_cast<double>(worldCorners.size());

  if(!std::isfinite(worldX) || !std::isfinite(worldY)) {
    Logger::error("SquareDetectionActionHandler:"
                  "実座標の計算結果が不正です");
    response.wasDetected = false;
    return;
  }

  if(worldX < 0.0 || worldX > BOARD_WIDTH || worldY < 0.0 || worldY > BOARD_HEIGHT) {
    Logger::printfLog(Logger::WARNING,
                      "SquareDetectionActionHandler: "
                      "変換結果が校正用紙の範囲外です "
                      "(%.2f, %.2f)",
                      worldX, worldY);
  }

  // =====================================================
  // 7. 前方距離・横方向距離を計算
  // =====================================================

  const double lateralDistance = worldX - BOARD_WIDTH / 2.0;

  const double forwardDistance = CAMERA_TO_BOARD_NEAR_EDGE + worldY;

  // =====================================================
  // 8. 回頭補正用角度を計算
  //
  // SquareAngleAdjustmentと同じ角度計算
  // =====================================================

  std::array<cv::Point, 4> corners = {
    detectionResult.topLeft,
    detectionResult.topRight,
    detectionResult.bottomRight,
    detectionResult.bottomLeft
  };

  // Y座標が小さい2点を選択
  std::sort(corners.begin(), corners.end(),
            [](const cv::Point& a, const cv::Point& b) {
              return a.y < b.y;
            });

  const auto& point1 = corners[0];
  const auto& point2 = corners[1];

  // X座標が小さい方を左上、大きい方を右上とする
  const auto& topLeft = point1.x < point2.x ? point1 : point2;
  const auto& topRight = point1.x < point2.x ? point2 : point1;

  const double deltaX = static_cast<double>(topRight.x - topLeft.x);
  const double deltaY = static_cast<double>(topRight.y - topLeft.y);

  const double angleRad = std::atan2(deltaY, deltaX);

  const double angleDeg = angleRad * 180.0 / std::acos(-1.0);

  // =====================================================
  // 9. 検出結果をレスポンスへ格納
  // =====================================================

  response.wasDetected = true;
  response.centerX = centerX;
  response.centerY = centerY;
  response.forwardDistance = forwardDistance;
  response.lateralDistance = lateralDistance;

  // =====================================================
  // 10. 計測終了
  // =====================================================

  const auto endTime = std::chrono::steady_clock::now();

  const double elapsedSeconds
      = std::chrono::duration<double>(endTime - startTime).count();

  const double captureMs
      = std::chrono::duration<double, std::milli>(captureEndTime - startTime).count();
  const double detectMs
      = std::chrono::duration<double, std::milli>(detectEndTime - captureEndTime).count();
  const double calculateMs
      = std::chrono::duration<double, std::milli>(endTime - detectEndTime).count();

  // 1行に3区間の時間をまとめて記録する
  Logger::printfLog(Logger::INFO,
                    "[SquareProfile] capture=%.2f ms detect=%.2f ms "
                    "calculate=%.2f ms total=%.2f ms",
                    captureMs, detectMs, calculateMs,
                    captureMs + detectMs + calculateMs);

  // =====================================================
  // 従来どおり距離・角度・合計時間を出力
  // =====================================================

  Logger::printfLog(Logger::INFO,
                    "[SquareTime] "
                    "time=%.4f sec "
                    "forward=%.2f mm "
                    "lateral=%.2f mm "
                    "angle=%.2f deg",
                    elapsedSeconds,
                    forwardDistance,
                    lateralDistance,
                    angleDeg);
}
