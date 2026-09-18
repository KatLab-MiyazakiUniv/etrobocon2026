/**
 * @file   SquareDetectionActionHandler.cpp
 * @brief  正方形を検出し、実距離を計算するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "SquareDetectionActionHandler.h"

namespace {

  /**
   * @brief 正方形検出前に取得するフレーム数
   * カメラバッファに残っている古い画像を破棄するために使用する。
   */
  constexpr int FRAME_CAPTURE_COUNT = 3;

  /**
   * @brief 画像座標を実際の距離[mm]へ変換するための校正用紙のサイズと中心位置を定義する。
   */
  constexpr double CALIBRATION_WIDTH = 196.0;   // 校正用紙の横幅[mm]
  constexpr double CALIBRATION_HEIGHT = 120.0;  // 校正用紙の縦幅[mm]
  constexpr double CALIBRATION_CENTER_X
      = CALIBRATION_WIDTH / 2.0;  // 校正用紙の左端から中心までの距離[mm]

  /**
   * @brief カメラ直下の床点から
   *        校正用紙手前側までの距離[mm]
   */
  constexpr double CAMERA_TO_BOARD_NEAR_EDGE = 200.0;

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
  /**
   * 実際のカメラ画像から取得した
   * 校正用紙4隅の画像座標[px]
   * 0: 左上
   * 1: 右上
   * 2: 右下
   * 3: 左下
   */
  const std::vector<cv::Point2f> imagePoints
      = { { 570.0f, 287.0f }, { 1390.0f, 287.0f }, { 1501.0f, 573.0f }, { 452.0f, 569.0f } };

  /**
   * 校正用紙上の実座標[mm]
   * 左上を(0,0)とする。
   */
  const std::vector<cv::Point2f> worldPoints
      = { { 0.0f, 0.0f },
          { static_cast<float>(CALIBRATION_WIDTH), 0.0f },
          { static_cast<float>(CALIBRATION_WIDTH), static_cast<float>(CALIBRATION_HEIGHT) },
          { 0.0f, static_cast<float>(CALIBRATION_HEIGHT) } };

  // カメラ画像の座標を実際の距離[mm]へ変換するためのホモグラフィ行列を作成する
  homography = cv::getPerspectiveTransform(imagePoints, worldPoints);

  Logger::info("SquareDetectionActionHandler: ホモグラフィ行列を初期化しました");
}

void SquareDetectionActionHandler::execute(const CameraServer::SquareDetectorRequest& request,
                                           CameraServer::SquareDetectorResponse& response)
{
  response = {};

  // カメラバッファの古いフレームを破棄し、最新のフレームで正方形を検出する。
  cv::Mat frame;

  for(int i = 0; i < FRAME_CAPTURE_COUNT; ++i) {
    cv::Mat capturedFrame;

    // フレームを取得し、取得に失敗した場合は検出処理を中止する。
    if(!camera.getFrame(capturedFrame)) {
      Logger::printfLog(Logger::ERROR,
                        "SquareDetectionActionHandler: "
                        "フレーム取得失敗 "
                        "%d / %d",
                        i + 1, FRAME_CAPTURE_COUNT);

      response.wasDetected = false;
      return;
    }

    Logger::printfLog(Logger::INFO,
                      "SquareDetectionActionHandler: "
                      "フレーム取得完了 "
                      "%d / %d",
                      i + 1, FRAME_CAPTURE_COUNT);

    frame = capturedFrame.clone();  // 3回目に取得したフレームだけ保持する
  }

  // 3回目に取得したフレームが空の場合は、検出処理を中止する。
  if(frame.empty()) {
    Logger::error("SquareDetectionActionHandler: "
                  "最終フレームが空です");

    response.wasDetected = false;
    return;
  }

  Logger::info("SquareDetectionActionHandler: "
               "3回目に取得したフレームを正方形検出に使用します");

  const cv::Rect localRoi(request.roi.x, request.roi.y, request.roi.width,
                          request.roi.height);  // リクエストで指定されたROIを使用する

  detector.setValidatedRoi(localRoi);

  // 4. 正方形検出
  BoundingBoxDetectionResult result{};

  detector.detect(frame, result);

  if(!result.wasDetected) {
    Logger::warning("SquareDetectionActionHandler: "
                    "正方形を検出できませんでした");

    response.wasDetected = false;

    return;
  }

  // 5. 4頂点をレスポンスへ格納

  response.wasDetected = true;
  response.corners[0].x = result.topLeft.x;
  response.corners[0].y = result.topLeft.y;
  response.corners[1].x = result.topRight.x;
  response.corners[1].y = result.topRight.y;
  response.corners[2].x = result.bottomRight.x;
  response.corners[2].y = result.bottomRight.y;
  response.corners[3].x = result.bottomLeft.x;
  response.corners[3].y = result.bottomLeft.y;

  // 6. 正方形中心座標
  const double centerX
      = (static_cast<double>(result.topLeft.x) + static_cast<double>(result.topRight.x)
         + static_cast<double>(result.bottomRight.x) + static_cast<double>(result.bottomLeft.x))
        / 4.0;

  const double centerY
      = (static_cast<double>(result.topLeft.y) + static_cast<double>(result.topRight.y)
         + static_cast<double>(result.bottomRight.y) + static_cast<double>(result.bottomLeft.y))
        / 4.0;

  // 7. pixel座標 -> 校正用紙上の実座標
  const cv::Point2f worldPoint = pixelToWorld(centerX, centerY);
  const double worldX = static_cast<double>(worldPoint.x);
  const double worldY = static_cast<double>(worldPoint.y);

  // 校正用紙の中心を基準に、正方形の横方向の距離を計算する。
  const double lateralDistance = worldX - CALIBRATION_CENTER_X;

  // カメラから正方形までの前方方向の距離を計算する。
  const double forwardDistance = CAMERA_TO_BOARD_NEAR_EDGE + CALIBRATION_HEIGHT - worldY;

  // レスポンスへ格納
  response.centerX = centerX;
  response.centerY = centerY;
  response.forwardDistance = forwardDistance;
  response.lateralDistance = lateralDistance;
  Logger::printfLog(Logger::INFO,
                    "SquareDetectionActionHandler: 正方形検出 "
                    "左上=(%d,%d) 右上=(%d,%d) "
                    "右下=(%d,%d) 左下=(%d,%d)",
                    result.topLeft.x, result.topLeft.y, result.topRight.x, result.topRight.y,
                    result.bottomRight.x, result.bottomRight.y, result.bottomLeft.x,
                    result.bottomLeft.y);

  Logger::printfLog(Logger::INFO, "SquareDetectionActionHandler: 正方形中心=(%.2f, %.2f)", centerX,
                    centerY);

  Logger::printfLog(Logger::INFO, "SquareDetectionActionHandler: 校正用紙上の座標=(%.2f, %.2f) mm",
                    worldX, worldY);

  Logger::printfLog(Logger::INFO,
                    "SquareDetectionActionHandler: 前方距離=%.2f mm 横方向距離=%.2f mm",
                    forwardDistance, lateralDistance);
}

cv::Point2f SquareDetectionActionHandler::pixelToWorld(double pixelX, double pixelY) const
{
  const std::vector<cv::Point2f> sourcePoints
      = { { static_cast<float>(pixelX), static_cast<float>(pixelY) } };

  std::vector<cv::Point2f> destinationPoints;

  cv::perspectiveTransform(sourcePoints, destinationPoints, homography);

  if(destinationPoints.empty()) {
    Logger::error("SquareDetectionActionHandler: "
                  "画像座標から実座標への変換に失敗しました");
    return { 0.0f, 0.0f };
  }

  return destinationPoints[0];
}