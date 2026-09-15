/**
 * @file   SquareAngleAdjustment.cpp
 * @brief  正方形の画像座標をホモグラフィ変換して、
 *         カメラ基準の実距離・補正角度を求めるクラス
 * @author yutaro-1214
 */

#include "SquareAngleAdjustment.h"

#include <cmath>
#include <vector>

#include "Logger.h"
#include "SocketClient.h"

namespace {

  /**
   * @brief radからdegへの変換係数
   */
  constexpr double SQUARE_RAD_TO_DEG = 180.0 / 3.14159265358979323846;

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
   *
   * 196 / 2 = 98
   */
  constexpr double CALIBRATION_CENTER_X = CALIBRATION_WIDTH / 2.0;

  /**
   * @brief カメラ直下の床点から
   *        校正用紙手前側までの距離[mm]
   *
   * 実測値。
   */
  constexpr double CAMERA_TO_BOARD_NEAR_EDGE = 200.0;

}  // namespace

SquareAngleAdjustment::SquareAngleAdjustment(Robot& _robot) : robot(_robot)
{
  initializeHomography();

  LOG_CREATE("SquareAngleAdjustment");
}

SquareAngleAdjustment::~SquareAngleAdjustment()
{
  LOG_DESTROY("SquareAngleAdjustment");
}

void SquareAngleAdjustment::initializeHomography()
{
  /*
   * 実際のカメラ画像から取得した
   * 校正用紙4隅の画像座標[px]
   *
   * 順番:
   *
   * 0: 左上
   * 1: 右上
   * 2: 右下
   * 3: 左下
   */
  const std::vector<cv::Point2f> imagePoints
      = { { 570.0f, 287.0f }, { 1390.0f, 287.0f }, { 1501.0f, 573.0f }, { 452.0f, 569.0f } };

  /*
   * 校正用紙上の実座標[mm]
   *
   * 左上を(0, 0)とする。
   *
   *      x
   *      →
   *
   * (0,0) ----------- (196,0)
   *   |
   *   |
   *   |
   * (0,120) -------- (196,120)
   *
   * yは下方向がプラス。
   */
  const std::vector<cv::Point2f> worldPoints
      = { { 0.0f, 0.0f }, { 196.0f, 0.0f }, { 196.0f, 120.0f }, { 0.0f, 120.0f } };

  homography = cv::getPerspectiveTransform(imagePoints, worldPoints);

  Logger::info("SquareAngleAdjustment: "
               "homography initialized");
}

SquareAngleAdjustment::Result SquareAngleAdjustment::calculate(
    const CameraServer::SquareDetectorRequest& request)
{
  Result result{};

  Logger::info("SquareAngleAdjustment: "
               "calculate CALLED");

  // =====================================================
  // 1. SocketClient取得
  // =====================================================

  SocketClient& client = robot.getCameraSocketClientInstance();

  CameraServer::SquareDetectorResponse response{};

  // =====================================================
  // 2. 正方形検出要求
  // =====================================================

  const bool success = client.executeSquareDetection(request, response);

  if(!success) {
    Logger::warning("SquareAngleAdjustment: "
                    "square detection communication failed");

    return result;
  }

  // =====================================================
  // 3. 正方形未検出
  // =====================================================

  if(!response.wasDetected) {
    Logger::warning("SquareAngleAdjustment: "
                    "square not detected");

    return result;
  }

  // =====================================================
  // 4. 正方形4隅から中心座標を求める
  // =====================================================

  double sumX = 0.0;
  double sumY = 0.0;

  for(const auto& corner : response.corners) {
    sumX += corner.x;
    sumY += corner.y;
  }

  const double centerX = sumX / static_cast<double>(CameraServer::SQUARE_CORNER_COUNT);

  const double centerY = sumY / static_cast<double>(CameraServer::SQUARE_CORNER_COUNT);

  // =====================================================
  // 5. pixel座標 -> 校正用紙上の実座標[mm]
  // =====================================================

  const cv::Point2f worldPoint = pixelToWorld(centerX, centerY);

  const double worldX = static_cast<double>(worldPoint.x);

  const double worldY = static_cast<double>(worldPoint.y);

  // =====================================================
  // 6. 校正用紙座標 -> カメラ基準座標
  // =====================================================

  /*
   * -----------------------------------------------------
   * 横方向
   * -----------------------------------------------------
   *
   * 校正用紙中央 x = 98mm を
   * カメラ正面とする。
   *
   * worldX = 98
   *   -> lateral = 0
   *
   * worldX < 98
   *   -> 左側なのでマイナス
   *
   * worldX > 98
   *   -> 右側なのでプラス
   */
  const double lateralDistance = worldX - CALIBRATION_CENTER_X;

  /*
   * -----------------------------------------------------
   * 前方距離
   * -----------------------------------------------------
   *
   * カメラ直下の床点から
   * 校正用紙手前側まで200mm。
   *
   * 校正用紙座標では、
   *
   * 上端:
   *   worldY = 0
   *
   * 下端:
   *   worldY = 120
   *
   * なので、
   *
   * 紙の下端:
   *
   * 200 + (120 - 120)
   * = 200mm
   *
   * 紙の中央:
   *
   * 200 + (120 - 60)
   * = 260mm
   *
   * 紙の上端:
   *
   * 200 + (120 - 0)
   * = 320mm
   */
  const double forwardDistance = CAMERA_TO_BOARD_NEAR_EDGE + CALIBRATION_HEIGHT - worldY;

  // =====================================================
  // 7. 補正角度
  // =====================================================

  const double correctionAngle = calculateCorrectionAngle(forwardDistance, lateralDistance);

  // =====================================================
  // 8. 正方形までの直線距離
  // =====================================================

  const double straightDistance = std::hypot(forwardDistance, lateralDistance);

  // =====================================================
  // 9. 結果格納
  // =====================================================

  result.wasDetected = true;

  result.centerX = centerX;

  result.centerY = centerY;

  result.worldX = worldX;

  result.worldY = worldY;

  result.forwardDistance = forwardDistance;

  result.lateralDistance = lateralDistance;

  result.correctionAngle = correctionAngle;

  result.straightDistance = straightDistance;

  // =====================================================
  // 10. ログ
  // =====================================================

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "pixel=(%.2f, %.2f)",
                    centerX, centerY);

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "world=(%.2f, %.2f) mm",
                    worldX, worldY);

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "forward=%.2f mm "
                    "lateral=%.2f mm",
                    forwardDistance, lateralDistance);

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "angle=%.2f deg "
                    "distance=%.2f mm",
                    correctionAngle, straightDistance);

  return result;
}

cv::Point2f SquareAngleAdjustment::pixelToWorld(double pixelX, double pixelY) const
{
  const std::vector<cv::Point2f> sourcePoints
      = { { static_cast<float>(pixelX), static_cast<float>(pixelY) } };

  std::vector<cv::Point2f> destinationPoints;

  cv::perspectiveTransform(sourcePoints, destinationPoints, homography);

  if(destinationPoints.empty()) {
    Logger::error("SquareAngleAdjustment: "
                  "perspectiveTransform failed");

    return { 0.0f, 0.0f };
  }

  return destinationPoints[0];
}

double SquareAngleAdjustment::calculateCorrectionAngle(double forwardDistance,
                                                       double lateralDistance) const
{
  if(forwardDistance <= 0.0) {
    Logger::warning("SquareAngleAdjustment: "
                    "invalid forward distance");

    return 0.0;
  }

  /*
   * lateral > 0
   *   -> 正方形が右側
   *
   * lateral < 0
   *   -> 正方形が左側
   */
  return std::atan2(lateralDistance, forwardDistance) * SQUARE_RAD_TO_DEG;
}