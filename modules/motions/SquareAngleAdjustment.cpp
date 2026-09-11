/**
 * @file   SquareAngleAdjustment.cpp
 * @brief  正方形の画像座標からロボット基準の角度・距離を計算するクラス
 * @author yutaro-1214
 */

#include "SquareAngleAdjustment.h"

#include <cmath>

#include "Logger.h"
#include "SocketClient.h"

namespace {

  /**
   * @brief カメラ画像中央X座標[px]
   */
  constexpr double IMAGE_CENTER_X = 960.0;

  /**
   * @brief radからdegへの変換係数
   */
  constexpr double SQUARE_RAD_TO_DEG
      = 180.0 / 3.14159265358979323846;

}  // namespace

SquareAngleAdjustment::SquareAngleAdjustment(
    Robot& _robot)
  : robot(_robot)
{
  LOG_CREATE(
      "SquareAngleAdjustment");
}

SquareAngleAdjustment::~SquareAngleAdjustment()
{
  LOG_DESTROY(
      "SquareAngleAdjustment");
}

SquareAngleAdjustment::Result
SquareAngleAdjustment::calculate(
    const CameraServer::SquareDetectorRequest& request)
{
  Result result {};

  Logger::info(
      "SquareAngleAdjustment: "
      "calculate CALLED");

  // =====================================================
  // 1. SocketClient取得
  // =====================================================

  SocketClient& client
      = robot.getCameraSocketClientInstance();

  CameraServer::SquareDetectorResponse response {};

  // =====================================================
  // 2. カメラサーバーへ検出要求
  // =====================================================

  const bool success
      = client.executeSquareDetection(
          request,
          response);

  if(!success) {

    Logger::warning(
        "SquareAngleAdjustment: "
        "square detection communication failed");

    return result;
  }

  if(!response.wasDetected) {

    Logger::warning(
        "SquareAngleAdjustment: "
        "square not detected");

    return result;
  }

  // =====================================================
  // 3. 正方形中心
  // =====================================================

  double sumX = 0.0;
  double sumY = 0.0;

  for(const auto& corner :
      response.corners) {

    sumX += corner.x;
    sumY += corner.y;
  }

  const double centerX
      = sumX
        / static_cast<double>(
            CameraServer::SQUARE_CORNER_COUNT);

  const double centerY
      = sumY
        / static_cast<double>(
            CameraServer::SQUARE_CORNER_COUNT);

  // =====================================================
  // 4. 画像座標 → 実距離
  // =====================================================

  const double forwardDistance
      = pixelYToForwardDistance(
          centerY);

  const double lateralDistance
      = pixelToLateralDistance(
          centerX,
          centerY);

  // =====================================================
  // 5. 補正角度
  // =====================================================

  const double correctionAngle
      = calculateCorrectionAngle(
          forwardDistance,
          lateralDistance);

  // =====================================================
  // 6. 正方形までの直線距離
  // =====================================================

  const double straightDistance
      = std::hypot(
          forwardDistance,
          lateralDistance);

  // =====================================================
  // 7. 結果
  // =====================================================

  result.wasDetected = true;

  result.centerX
      = centerX;

  result.centerY
      = centerY;

  result.forwardDistance
      = forwardDistance;

  result.lateralDistance
      = lateralDistance;

  result.correctionAngle
      = correctionAngle;

  result.straightDistance
      = straightDistance;

  // =====================================================
  // 8. ログ
  // =====================================================

  Logger::printfLog(
      Logger::INFO,
      "SquareAngleAdjustment: "
      "center=(%.2f, %.2f)",
      centerX,
      centerY);

  Logger::printfLog(
      Logger::INFO,
      "SquareAngleAdjustment: "
      "forward=%.2f mm "
      "lateral=%.2f mm",
      forwardDistance,
      lateralDistance);

  Logger::printfLog(
      Logger::INFO,
      "SquareAngleAdjustment: "
      "angle=%.2f deg "
      "distance=%.2f mm",
      correctionAngle,
      straightDistance);

  return result;
}

double
SquareAngleAdjustment::pixelYToForwardDistance(
    double y) const
{
  return
      (-0.153821 * y + 677.174)
      / (0.00144003 * y + 1.0);
}

double
SquareAngleAdjustment::pixelToLateralDistance(
    double x,
    double y) const
{
  const double pixelsPer12mm
      = 0.04893 * y
        + 34.414;

  if(pixelsPer12mm <= 0.0) {

    Logger::error(
        "SquareAngleAdjustment: "
        "invalid lateral conversion");

    return 0.0;
  }

  return
      12.0
      * (x - IMAGE_CENTER_X)
      / pixelsPer12mm;
}

double
SquareAngleAdjustment::calculateCorrectionAngle(
    double forwardDistance,
    double lateralDistance) const
{
  if(forwardDistance <= 0.0) {

    Logger::warning(
        "SquareAngleAdjustment: "
        "invalid forward distance");

    return 0.0;
  }

  return
      std::atan2(
          lateralDistance,
          forwardDistance)
      * SQUARE_RAD_TO_DEG;
}