/**
 * @file   SquareDetectionActionHandler.cpp
 * @brief  正方形検出要求を処理し、
 *         画像座標を実距離へ変換するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "SquareDetectionActionHandler.h"

namespace {

  /**
   * @brief 正方形検出前に取得するフレーム数
   */
  constexpr int FRAME_CAPTURE_COUNT = 10;

  /**
   * @brief 校正用紙の横幅[mm]
   */
  constexpr double CALIBRATION_WIDTH = 196.0;

  /**
   * @brief 校正用紙の縦幅[mm]
   */
  constexpr double CALIBRATION_HEIGHT = 120.0;

  /**
   * @brief 校正用紙中央X座標[mm]
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

SquareDetectionActionHandler::
    SquareDetectionActionHandler(
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

  LOG_CREATE("SquareDetectionActionHandler");
}

SquareDetectionActionHandler::
    ~SquareDetectionActionHandler()
{
  LOG_DESTROY("SquareDetectionActionHandler");
}

void SquareDetectionActionHandler::
    initializeHomography()
{
  /**
   * 実際のカメラ画像から取得した
   * 校正用紙4隅の画像座標[px]
   * 0: 左上
   * 1: 右上
   * 2: 右下
   * 3: 左下
   */
  const std::vector<cv::Point2f> imagePoints = {
    { 570.0f, 287.0f },
    { 1390.0f, 287.0f },
    { 1501.0f, 573.0f },
    { 452.0f, 569.0f }
  };

  /**
   * 校正用紙上の実座標[mm]
   * 左上を(0,0)とする。
   */
  const std::vector<cv::Point2f> worldPoints = {
    {
      0.0f,
      0.0f
    },
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

  homography
      = cv::getPerspectiveTransform(
          imagePoints,
          worldPoints);

  Logger::info("SquareDetectionActionHandler: ホモグラフィを初期化しました");
}

void SquareDetectionActionHandler::execute(
    const CameraServer::SquareDetectorRequest& request,
    CameraServer::SquareDetectorResponse& response)
{
  response = {};
  static unsigned long detectionCount = 0;

  ++detectionCount;

 Logger::printfLog(
    Logger::INFO,
    "SquareDetectionActionHandler: 検出開始 #%lu",
    detectionCount);

  //カメラフレームを複数回取得
  cv::Mat frame;

  for(int i = 0;
      i < FRAME_CAPTURE_COUNT;
      ++i) {

    cv::Mat capturedFrame;

    if(!camera.getFrame(capturedFrame)) {

      Logger::printfLog(
          Logger::ERROR,
          "SquareDetectionActionHandler: "
          "フレーム取得失敗 "
          "%d / %d",
          i + 1,
          FRAME_CAPTURE_COUNT);
      response.wasDetected = false;

      return;
    }

    const cv::Scalar meanValue
        = cv::mean(capturedFrame);

  Logger::printfLog(
    Logger::INFO,
    "SquareDetectionActionHandler: "
    "検出=%lu "
    "フレーム=%d/%d "
    "平均=(%.2f, %.2f, %.2f)",
    detectionCount,
    i + 1,
    FRAME_CAPTURE_COUNT,
    meanValue[0],
    meanValue[1],
    meanValue[2]);

    // 最後に取得したフレームを使用する
    frame = capturedFrame.clone();
  }

  // 2. 最終フレーム確認
  if(frame.empty()) {

    Logger::error(
        "SquareDetectionActionHandler: "
        "最終フレームが空です");

    response.wasDetected = false;

    return;
  }

  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "detection=%lu "
      "last frame is used for detection",
      detectionCount);

  //調査用画像保存
  const std::string debugImagePath
      = "/tmp/square_detection_"
        + std::to_string(detectionCount)
        + ".jpg";

  const bool imageSaved
      = cv::imwrite(
          debugImagePath,
          frame);

  if(imageSaved) {

    Logger::printfLog(
        Logger::INFO,
        "SquareDetectionActionHandler: "
        "debug image saved: %s",
        debugImagePath.c_str());

  } else {

    Logger::printfLog(
        Logger::WARNING,
        "SquareDetectionActionHandler: "
        "debug image save failed: %s",
        debugImagePath.c_str());
  }

  // 4. ROI設定
  const cv::Rect localRoi(
      request.roi.x,
      request.roi.y,
      request.roi.width,
      request.roi.height);

Logger::printfLog(
    Logger::INFO,
    "SquareDetectionActionHandler: "
    "ROI=(%d,%d,%d,%d)",
    request.roi.x,
    request.roi.y,
    request.roi.width,
    request.roi.height);

  detector.setValidatedRoi(
      localRoi);

  // 5. 正方形検出
  BoundingBoxDetectionResult result{};

  detector.detect(
      frame,
      result);

  if(!result.wasDetected) {

    Logger::printfLog(
        Logger::WARNING,
        "SquareDetectionActionHandler: "
        "DETECTION #%lu "
        "正方形を検出できませんでした",
        detectionCount);

    response.wasDetected = false;

    return;
  }

  // 6. 4頂点をレスポンスへ格納
  response.wasDetected = true;

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

  //正方形中心座標
  const double centerX
      = (
          static_cast<double>(
              result.topLeft.x)
          + static_cast<double>(
              result.topRight.x)
          + static_cast<double>(
              result.bottomRight.x)
          + static_cast<double>(
              result.bottomLeft.x)
        )
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
              result.bottomLeft.y)
        )
        / 4.0;

  // 8. pixel座標 -> 校正用紙上の実座標
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

  //横方向距離
  const double lateralDistance
      = worldX
        - CALIBRATION_CENTER_X;

  // 10. 前方距離
  const double forwardDistance
      = CAMERA_TO_BOARD_NEAR_EDGE
        + CALIBRATION_HEIGHT
        - worldY;

  //レスポンスへ格納
  response.centerX
      = centerX;
  response.centerY
      = centerY;
  response.forwardDistance
      = forwardDistance;
  response.lateralDistance
      = lateralDistance;

  // 12. ログ
  Logger::printfLog(
      Logger::INFO,
      "SquareDetectionActionHandler: "
      "DETECTION #%lu "
      "square detected "
      "TL=(%d,%d) "
      "TR=(%d,%d) "
      "BR=(%d,%d) "
      "BL=(%d,%d)",
      detectionCount,
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
    "検出 #%lu "
    "中心=(%.2f, %.2f)",
    detectionCount,
    centerX,
    centerY);

Logger::printfLog(
    Logger::INFO,
    "SquareDetectionActionHandler: "
    "検出 #%lu "
    "実座標=(%.2f, %.2f) mm",
    detectionCount,
    worldX,
    worldY);

 Logger::printfLog(
    Logger::INFO,
    "SquareDetectionActionHandler: "
    "検出 #%lu "
    "前方=%.2f mm "
    "横方向=%.2f mm",
    detectionCount,
    forwardDistance,
    lateralDistance);

  Logger::printfLog(
    Logger::INFO,
    "SquareDetectionActionHandler: "
    "検出 #%lu 完了",
    detectionCount);
}

cv::Point2f
SquareDetectionActionHandler::pixelToWorld(
    double pixelX,
    double pixelY) const
{
  const std::vector<cv::Point2f> sourcePoints = {
    {
      static_cast<float>(pixelX),
      static_cast<float>(pixelY)
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
    "透視変換に失敗しました");

    return {
      0.0f,
      0.0f
    };
  }

  return destinationPoints[0];
}