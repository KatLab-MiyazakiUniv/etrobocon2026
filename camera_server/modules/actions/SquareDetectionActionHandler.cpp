/**
 * @file   SquareDetectionActionHandler.cpp
 * @brief  正方形検出要求を処理し、
 *         画像座標を実距離へ変換するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "SquareDetectionActionHandler.h"

#include <vector>

namespace {

  /**
   * @brief 校正用紙の横幅[mm]
   */
  constexpr double CALIBRATION_WIDTH
      = 196.0;

  /**
   * @brief 校正用紙の縦幅[mm]
   */
  constexpr double CALIBRATION_HEIGHT
      = 120.0;

  /**
   * @brief 校正用紙中央X座標[mm]
   *
   * 196 / 2 = 98mm
   */
  constexpr double CALIBRATION_CENTER_X
      = CALIBRATION_WIDTH / 2.0;

  /**
   * @brief カメラ直下の床点から
   *        校正用紙手前側までの距離[mm]
   */
  constexpr double CAMERA_TO_BOARD_NEAR_EDGE
      = 200.0;

}  // namespace

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
  initializeHomography();

  LOG_CREATE(
      "SquareDetectionActionHandler");
}

SquareDetectionActionHandler::~SquareDetectionActionHandler()
{
  LOG_DESTROY(
      "SquareDetectionActionHandler");
}

void SquareDetectionActionHandler::initializeHomography()
{
  // =====================================================
  // カメラ画像上の校正用紙4隅
  //
  // 順番:
  //
  // 左上
  // 右上
  // 右下
  // 左下
  // =====================================================

  const std::vector<cv::Point2f> imagePoints
      = {
          { 570.0f, 287.0f },
          { 1390.0f, 287.0f },
          { 1501.0f, 573.0f },
          { 452.0f, 569.0f }
        };

  // =====================================================
  // 校正用紙上の実座標[mm]
  //
  // 左上を(0, 0)とする
  //
  // (0,0) -------- (196,0)
  //   |                |
  //   |                |
  // (0,120) ------ (196,120)
  // =====================================================

  const std::vector<cv::Point2f> worldPoints
      = {
          { 0.0f, 0.0f },
          {
              static_cast<float>(
                  CALIBRATION_WIDTH),
              0.0f
          },
          {
              static_cast<float>(
                  CALIBRATION_WIDTH),
              static_cast<float>(
                  CALIBRATION_HEIGHT)
          },
          {
              0.0f,
              static_cast<float>(
                  CALIBRATION_HEIGHT)
          }
        };

  // =====================================================
  // ホモグラフィ行列生成
  // =====================================================

  homography
      = cv::getPerspectiveTransform(
          imagePoints,
          worldPoints);

  Logger::info(
      "SquareDetectionActionHandler: "
      "homography initialized");
}

void SquareDetectionActionHandler::execute(
    const CameraServer::SquareDetectorRequest& request,
    CameraServer::SquareDetectorResponse& response)
{
  response = {};

  // =====================================================
  // 1. カメラフレーム取得
  // =====================================================

  cv::Mat frame;

  if(!camera.getFrame(
         frame)) {

    Logger::error(
        "SquareDetectionActionHandler: "
        "フレームの取得に失敗しました");

    response.wasDetected
        = false;

    return;
  }

  // =====================================================
  // 2. ROI設定
  // =====================================================

  const cv::Rect localRoi(
      request.roi.x,
      request.roi.y,
      request.roi.width,
      request.roi.height);

  detector.setValidatedRoi(
      localRoi);

  // =====================================================
  // 3. 正方形検出
  // =====================================================

  BoundingBoxDetectionResult result {};

  detector.detect(
      frame,
      result);

  if(!result.wasDetected) {

    Logger::warning(
        "SquareDetectionActionHandler: "
        "正方形を検出できませんでした");

    response.wasDetected
        = false;

    return;
  }

  // =====================================================
  // 4. 4頂点をレスポンスへ格納
  // =====================================================

  response.wasDetected
      = true;

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

  // =====================================================
  // 5. 正方形中心座標
  // =====================================================

  const double centerX
      = (
          static_cast<double>(
              result.topLeft.x)
          + static_cast<double>(
              result.topRight.x)
          + static_cast<double>(
              result.bottomRight.x)
          + static_cast<double>(
              result.bottomLeft.x))
        / 4.0;

  const double centerY
      = (
          static_cast<double>(
              result.topLeft.y)
          + static_cast<double>(
              result.topRight.y)
          + static_cast<double>(
              result.bottomRight.y)
          + static_cast<double>(
              result.bottomLeft.y))
        / 4.0;

  // =====================================================
  // 6. pixel座標 -> 校正用紙上の実座標
  // =====================================================

  const cv::Point2f worldPoint
      = pixelToWorld(
          centerX,
          centerY);

  const double worldX
      = static_cast<double>(
          worldPoint.x);

  const double worldY
      = static_cast<double>(
          worldPoint.y);

  // =====================================================
  // 7. 横方向距離
  //
  // 校正用紙中央をカメラ正面とする
  //
  // 左:
  //   マイナス
  //
  // 右:
  //   プラス
  // =====================================================

  const double lateralDistance
      = worldX
        - CALIBRATION_CENTER_X;

  // =====================================================
  // 8. 前方距離
  //
  // 校正用紙下端:
  //   200mm
  //
  // 校正用紙上端:
  //   320mm
  // =====================================================

  const double forwardDistance
      = CAMERA_TO_BOARD_NEAR_EDGE
        + CALIBRATION_HEIGHT
        - worldY;

  // =====================================================
  // 9. レスポンスへ格納
  // =====================================================

  response.centerX
      = centerX;

  response.centerY
      = centerY;

  response.forwardDistance
      = forwardDistance;

  response.lateralDistance
      = lateralDistance;

  // =====================================================
  // 10. ログ
  // =====================================================

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

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "center=(%.2f, %.2f)",
      centerX,
      centerY);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "world=(%.2f, %.2f) mm",
      worldX,
      worldY);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "forward=%.2f mm "
      "lateral=%.2f mm",
      forwardDistance,
      lateralDistance);
}

cv::Point2f
SquareDetectionActionHandler::pixelToWorld(
    double pixelX,
    double pixelY) const
{
  const std::vector<cv::Point2f> sourcePoints
      = {
          {
              static_cast<float>(
                  pixelX),
              static_cast<float>(
                  pixelY)
          }
        };

  std::vector<cv::Point2f>
      destinationPoints;

  cv::perspectiveTransform(
      sourcePoints,
      destinationPoints,
      homography);

  if(destinationPoints.empty()) {

    Logger::error(
        "SquareDetectionActionHandler: "
        "perspectiveTransform failed");

    return {
        0.0f,
        0.0f
    };
  }

  return destinationPoints[0];
}