/**
 * @file   SquareDetectionActionHandler.cpp
 * @brief  正方形検出要求を処理し、検出結果を実距離へ変換するクラス
 * @author yutaro-1214
 */

#include "SquareDetectionActionHandler.h"

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

    cv::Point2f(674.65F, 699.43F),  // (50, 50)
    cv::Point2f(970.28F, 698.92F),  // (100, 50)
    cv::Point2f(1265.69F, 698.89F)  // (150, 50)
  };

  /**
   * @brief 校正用紙上の基準点[mm]
   * 校正用紙左下を原点とする。
   */
  const std::vector<cv::Point2f> WORLD_POINTS
      = { cv::Point2f(50.0F, 150.0F), cv::Point2f(100.0F, 150.0F), cv::Point2f(150.0F, 150.0F),
          cv::Point2f(50.0F, 100.0F), cv::Point2f(150.0F, 100.0F), cv::Point2f(50.0F, 50.0F),
          cv::Point2f(100.0F, 50.0F), cv::Point2f(150.0F, 50.0F) };

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
   * 実際に設置した距離に合わせて変更すること。
   */
  constexpr double CAMERA_TO_BOARD_NEAR_EDGE = 160.0;

  /**
   * @brief 1回の検出で取得するフレーム数
   * カメラに残っている古いフレームを避けるため、
   * 複数枚取得して最後の画像を使用する。
   */
  constexpr int FRAME_CAPTURE_COUNT = 10;

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

  // 8点の基準点から画像座標を実座標へ変換するホモグラフィ行列を求める
  // 第3引数を0にしてRANSACを使用せず、全ての点を使って計算する
  homography = cv::findHomography(IMAGE_POINTS, WORLD_POINTS, 0);

  if(homography.empty()) {
    Logger::error("SquareDetectionActionHandler:"
                  "ホモグラフィ行列の計算に失敗しました");
    return;
  }

  /*
   * 計算精度を保つため、double型へ変換する。
   */
  homography.convertTo(homography, CV_64F);
  Logger::printfLog(Logger::INFO, "SquareDetectionActionHandler: "
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
  response = {};
  ++detectionCount;

  Logger::printfLog(Logger::INFO, "SquareDetectionActionHandler: 検出開始 #%d", detectionCount);

  if(homography.empty()) {
    Logger::error("SquareDetectionActionHandler:"
                  "ホモグラフィ行列が初期化されていません");
    response.wasDetected = false;
    return;
  }

  /*
   * カメラに残っている古いフレームを避けるため、
   * 10フレーム取得して最後のフレームを使用する。
   */
  cv::Mat frame;

  for(int i = 0; i < FRAME_CAPTURE_COUNT; ++i) {
    cv::Mat currentFrame;

    if(!camera.getFrame(currentFrame)) {
      Logger::error("SquareDetectionActionHandler:"
                    "フレームの取得に失敗しました");
      response.wasDetected = false;
      return;
    }

    const cv::Scalar meanValue = cv::mean(currentFrame);

    Logger::printfLog(Logger::INFO,
                      "SquareDetectionActionHandler: "
                      "検出=%d "
                      "フレーム=%d/%d "
                      "平均=(%.2f, %.2f, %.2f)",
                      detectionCount, i + 1, FRAME_CAPTURE_COUNT, meanValue[0], meanValue[1],
                      meanValue[2]);

    frame = currentFrame.clone();
  }

  /*
   * ROI設定
   */
  const cv::Rect roi(request.roi.x, request.roi.y, request.roi.width, request.roi.height);
  detector.setValidatedRoi(roi);
  Logger::printfLog(Logger::INFO,
                    "SquareDetectionActionHandler: "
                    "ROI=(%d,%d,%d,%d)",
                    request.roi.x, request.roi.y, request.roi.width, request.roi.height);

  /*
   * 正方形検出
   */
  BoundingBoxDetectionResult detectionResult{};
  detector.detect(frame, detectionResult);

  if(!detectionResult.wasDetected) {
    Logger::printfLog(Logger::WARNING,
                      "SquareDetectionActionHandler: "
                      "検出 #%d "
                      "正方形を検出できませんでした",
                      detectionCount);
    response.wasDetected = false;
    return;
  }

  Logger::printfLog(Logger::INFO,
                    "SquareDetectionActionHandler: "
                    "検出 #%d "
                    "正方形を検出しました "
                    "左上=(%d,%d) "
                    "右上=(%d,%d) "
                    "右下=(%d,%d) "
                    "左下=(%d,%d)",
                    detectionCount, detectionResult.topLeft.x, detectionResult.topLeft.y,
                    detectionResult.topRight.x, detectionResult.topRight.y,
                    detectionResult.bottomRight.x, detectionResult.bottomRight.y,
                    detectionResult.bottomLeft.x, detectionResult.bottomLeft.y);

  /*
   * 画像上の正方形中心座標
   */
  const double centerX = (detectionResult.topLeft.x + detectionResult.topRight.x
                          + detectionResult.bottomRight.x + detectionResult.bottomLeft.x)
                         / 4.0;

  const double centerY = (detectionResult.topLeft.y + detectionResult.topRight.y
                          + detectionResult.bottomRight.y + detectionResult.bottomLeft.y)
                         / 4.0;

  Logger::printfLog(Logger::INFO,
                    "SquareDetectionActionHandler: "
                    "検出 #%d "
                    "画像中心=(%.2f, %.2f)",
                    detectionCount, centerX, centerY);

  /*
   * 検出した4頂点をそれぞれ実座標へ変換する
   */
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

  Logger::printfLog(Logger::INFO,
                    "SquareDetectionActionHandler: "
                    "校正用紙座標=(%.2f, %.2f) mm",
                    worldX, worldY);

  if(worldX < 0.0 || worldX > BOARD_WIDTH || worldY < 0.0 || worldY > BOARD_HEIGHT) {
    Logger::printfLog(Logger::WARNING,
                      "SquareDetectionActionHandler: "
                      "変換結果が校正用紙の範囲外です "
                      "(%.2f, %.2f)",
                      worldX, worldY);
  }

  // 横方向距離
  // 校正用紙中央を0mmとして左右の距離を計算する
  const double lateralDistance = worldX - BOARD_WIDTH / 2.0;

  // 前方距離
  // カメラから正方形までの前方距離を計算する
  const double forwardDistance = CAMERA_TO_BOARD_NEAR_EDGE + worldY;

  Logger::printfLog(Logger::INFO,
                    "SquareDetectionActionHandler: "
                    "検出 #%d "
                    "前方=%.2f mm "
                    "横方向=%.2f mm",
                    detectionCount, forwardDistance, lateralDistance);

  response.wasDetected = true;
  response.centerX = centerX;
  response.centerY = centerY;
  response.forwardDistance = forwardDistance;
  response.lateralDistance = lateralDistance;

  Logger::info("SquareDetectionActionHandler: 検出終了");
}