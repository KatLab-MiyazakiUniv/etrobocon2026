/**
 * @file   SquareDetectionActionHandler.cpp
 * @brief  正方形検出結果を実座標へ変換するアクションハンドラクラス
 * @author yutaro-1214
 */

#include "SquareDetectionActionHandler.h"

#include <opencv2/opencv.hpp>

#include "Logger.h"

namespace {

  /**
   * @brief 画像上の基準点
   *
   * 4点の画像座標と実世界座標の対応から
   * ホモグラフィ行列を計算する。
   */
  const std::vector<cv::Point2f> IMAGE_POINTS = {
      cv::Point2f(570.0F, 287.0F),
      cv::Point2f(1390.0F, 287.0F),
      cv::Point2f(1501.0F, 573.0F),
      cv::Point2f(452.0F, 569.0F)
  };

  /**
   * @brief 基準領域の横幅[mm]
   */
  constexpr double BOARD_WIDTH = 196.0;

  /**
   * @brief 基準領域の奥行き[mm]
   */
  constexpr double BOARD_HEIGHT = 120.0;

  /**
   * @brief カメラから基準領域手前端までの距離[mm]
   */
  constexpr double CAMERA_TO_BOARD_NEAR_EDGE = 200.0;

  /**
   * @brief 実世界座標
   *
   * 左上  : (0, 0)
   * 右上  : (196, 0)
   * 右下  : (196, 120)
   * 左下  : (0, 120)
   */
  const std::vector<cv::Point2f> WORLD_POINTS = {
      cv::Point2f(0.0F, 0.0F),
      cv::Point2f(
          static_cast<float>(BOARD_WIDTH),
          0.0F),
      cv::Point2f(
          static_cast<float>(BOARD_WIDTH),
          static_cast<float>(BOARD_HEIGHT)),
      cv::Point2f(
          0.0F,
          static_cast<float>(BOARD_HEIGHT))
  };

  /**
   * @brief 1回の検出で取得するフレーム数
   *
   * カメラの古いフレームを避けるため、
   * 複数枚取得して最後の画像を使用する。
   */
  constexpr int FRAME_CAPTURE_COUNT = 10;

  /**
   * @brief 検出回数
   */
  int detectionCount = 0;

}  // namespace

SquareDetectionActionHandler::SquareDetectionActionHandler(CameraCapture& _camera)
  : camera(_camera),
    detector(cv::Rect(0, 0, CAM_MAX_WIDTH, CAM_MAX_HEIGHT))
{
  // =====================================================
  // ホモグラフィ行列作成
  // =====================================================

  homography
      = cv::getPerspectiveTransform(
          IMAGE_POINTS,
          WORLD_POINTS);

  Logger::info(
      "SquareDetectionActionHandler: "
      "homography initialized");

  LOG_CREATE("SquareDetectionActionHandler");
}

SquareDetectionActionHandler::~SquareDetectionActionHandler()
{
  LOG_DESTROY("SquareDetectionActionHandler");
}

void SquareDetectionActionHandler::execute(
    const CameraServer::SquareDetectorRequest& request,
    CameraServer::SquareDetectorResponse& response)
{
  // =====================================================
  // 初期化
  // =====================================================

  response = {};

  ++detectionCount;

  Logger::info("========================================");

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "DETECTION #%d START",
      detectionCount);

  Logger::info("========================================");

  // =====================================================
  // 1. カメラ画像取得
  //
  // 古いフレームを使用する可能性を減らすため、
  // 10フレーム取得して最後のフレームを使用する。
  // =====================================================

  cv::Mat frame;

  for(int i = 0; i < FRAME_CAPTURE_COUNT; ++i) {

    cv::Mat currentFrame;

    if(!camera.getFrame(currentFrame)) {

      Logger::error(
          "SquareDetectionActionHandler:"
          "フレームの取得に失敗しました");

      response.wasDetected = false;

      return;
    }

    // ---------------------------------------------------
    // フレーム確認用ログ
    // ---------------------------------------------------

    const cv::Scalar meanValue
        = cv::mean(currentFrame);

    Logger::printfLog(
        Logger::INFO,
        "SquareDetectionActionHandler: "
        "detection=%d "
        "frame=%d/%d "
        "mean=(%.2f, %.2f, %.2f)",
        detectionCount,
        i + 1,
        FRAME_CAPTURE_COUNT,
        meanValue[0],
        meanValue[1],
        meanValue[2]);

    // 最後に取得したフレームを保持
    frame = currentFrame.clone();
  }

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "detection=%d "
      "last frame is used for detection",
      detectionCount);

  // =====================================================
  // square_detection_N.jpg の保存は行わない
  //
  // 画像保存はSquareDetector側で
  //
  //   square_N_01_original.jpg
  //   square_N_02_processed.jpg
  //
  // の2枚のみ行う。
  // =====================================================

  // =====================================================
  // 2. ROI設定
  // =====================================================

  cv::Rect roi(
      request.roi.x,
      request.roi.y,
      request.roi.width,
      request.roi.height);

  detector.setValidatedRoi(roi);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "ROI=(%d,%d,%d,%d)",
      request.roi.x,
      request.roi.y,
      request.roi.width,
      request.roi.height);

  // =====================================================
  // 3. 正方形検出
  // =====================================================

  BoundingBoxDetectionResult detectionResult{};

  detector.detect(
      frame,
      detectionResult);

  // =====================================================
  // 4. 未検出
  // =====================================================

  if(!detectionResult.wasDetected) {

    Logger::printfLog(
        Logger::WARNING,
        "SquareDetectionActionHandler: "
        "DETECTION #%d "
        "正方形を検出できませんでした",
        detectionCount);

    response.wasDetected = false;

    return;
  }

  // =====================================================
  // 5. 検出した4頂点
  // =====================================================

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "DETECTION #%d "
      "square detected "
      "TL=(%d,%d) "
      "TR=(%d,%d) "
      "BR=(%d,%d) "
      "BL=(%d,%d)",
      detectionCount,
      detectionResult.topLeft.x,
      detectionResult.topLeft.y,
      detectionResult.topRight.x,
      detectionResult.topRight.y,
      detectionResult.bottomRight.x,
      detectionResult.bottomRight.y,
      detectionResult.bottomLeft.x,
      detectionResult.bottomLeft.y);

  // =====================================================
  // 6. 正方形中心座標
  // =====================================================

  const double centerX
      = (
            detectionResult.topLeft.x
            + detectionResult.topRight.x
            + detectionResult.bottomRight.x
            + detectionResult.bottomLeft.x)
        / 4.0;

  const double centerY
      = (
            detectionResult.topLeft.y
            + detectionResult.topRight.y
            + detectionResult.bottomRight.y
            + detectionResult.bottomLeft.y)
        / 4.0;

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "DETECTION #%d "
      "center=(%.2f, %.2f)",
      detectionCount,
      centerX,
      centerY);

  // =====================================================
  // 7. ホモグラフィ変換
  // =====================================================

  std::vector<cv::Point2f> imageCenter = {
      cv::Point2f(
          static_cast<float>(centerX),
          static_cast<float>(centerY))
  };

  std::vector<cv::Point2f> worldCenter;

  cv::perspectiveTransform(
      imageCenter,
      worldCenter,
      homography);

  if(worldCenter.empty()) {

    Logger::warning(
        "SquareDetectionActionHandler:"
        "ホモグラフィ変換に失敗しました");

    response.wasDetected = false;

    return;
  }

  const double worldX
      = static_cast<double>(
          worldCenter[0].x);

  const double worldY
      = static_cast<double>(
          worldCenter[0].y);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "DETECTION #%d "
      "world=(%.2f, %.2f) mm",
      detectionCount,
      worldX,
      worldY);

  // =====================================================
  // 8. ロボット基準の前後・左右距離計算
  //
  // worldX:
  //   ホモグラフィ基準領域の左端からの距離
  //
  // worldY:
  //   基準領域の奥側から手前側への距離
  //
  // 横方向:
  //   基準領域中央を0mmとする。
  //
  // 前方向:
  //   カメラから基準領域手前端まで200mm、
  //   基準領域奥行き120mmとして求める。
  // =====================================================

  const double lateralDistance
      = worldX
        - BOARD_WIDTH / 2.0;

  const double forwardDistance
      = CAMERA_TO_BOARD_NEAR_EDGE
        + BOARD_HEIGHT
        - worldY;

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "DETECTION #%d "
      "forward=%.2f mm "
      "lateral=%.2f mm",
      detectionCount,
      forwardDistance,
      lateralDistance);

  // =====================================================
  // 9. レスポンス設定
  //
  // このforwardDistanceはカメラ基準。
  //
  // SquareAngleAdjustment側で
  // ROBOT_TO_CAMERA_OFFSET = 100mm
  // を加えてロボット基準に補正する。
  // =====================================================

  response.wasDetected = true;

  response.centerX
      = centerX;

  response.centerY
      = centerY;

  response.forwardDistance
      = forwardDistance;

  response.lateralDistance
      = lateralDistance;

  // =====================================================
  // 終了ログ
  // =====================================================

  Logger::info("========================================");

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "DETECTION #%d FINISHED",
      detectionCount);

  Logger::info("========================================");
}