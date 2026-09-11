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
   *
   * 画像幅1920pxなので中央は960px。
   */
  constexpr double IMAGE_CENTER_X = 960.0;

  /**
   * @brief radからdegへの変換係数
   *
   * プロジェクト内に同名のRAD_TO_DEGが存在する可能性があるため、
   * SquareAngleAdjustment専用の名前にしている。
   */
  constexpr double SQUARE_RAD_TO_DEG = 180.0 / 3.14159265358979323846;

}  // namespace

SquareAngleAdjustment::SquareAngleAdjustment(Robot& _robot) : robot(_robot)
{
  LOG_CREATE("SquareAngleAdjustment");
}

SquareAngleAdjustment::~SquareAngleAdjustment()
{
  LOG_DESTROY("SquareAngleAdjustment");
}

SquareAngleAdjustment::Result SquareAngleAdjustment::calculate(
    const CameraServer::SquareDetectorRequest& request)
{
  Result result{};

  // =====================================================
  // 1. 正方形検出
  // =====================================================

  SocketClient& client = robot.getCameraSocketClientInstance();

  CameraServer::SquareDetectorResponse response{};

  const bool success = client.executeSquareDetection(request, response);

  if(!success) {
    Logger::warning("SquareAngleAdjustment: "
                    "square detection communication failed");

    return result;
  }

  if(!response.wasDetected) {
    Logger::warning("SquareAngleAdjustment: "
                    "square not detected");

    return result;
  }

  // =====================================================
  // 2. 四隅から正方形中心を計算
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
  // 3. 画像Y座標 → 前方距離
  // =====================================================

  const double forwardDistance = pixelYToForwardDistance(centerY);

  // =====================================================
  // 4. 画像X,Y座標 → 横方向距離
  // =====================================================

  const double lateralDistance = pixelToLateralDistance(centerX, centerY);

  // =====================================================
  // 5. 補正角度を計算
  // =====================================================

  const double correctionAngle = calculateCorrectionAngle(forwardDistance, lateralDistance);

  // =====================================================
  // 6. 回頭後の直進距離を計算
  //
  //          square
  //             *
  //            /|
  //           / |
  //          /  | forward
  //         /   |
  //    robot----+
  //       lateral
  //
  // straightDistance
  //   = sqrt(forward^2 + lateral^2)
  // =====================================================

  const double straightDistance = std::hypot(forwardDistance, lateralDistance);

  // =====================================================
  // 7. 結果
  // =====================================================

  result.wasDetected = true;

  result.centerX = centerX;

  result.centerY = centerY;

  result.forwardDistance = forwardDistance;

  result.lateralDistance = lateralDistance;

  result.correctionAngle = correctionAngle;

  result.straightDistance = straightDistance;

  // =====================================================
  // 8. ログ
  // =====================================================

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "center=(%.2f, %.2f)",
                    centerX, centerY);

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

double SquareAngleAdjustment::pixelYToForwardDistance(double y) const
{
  /*
   * 校正条件
   *
   * カメラ解像度:
   *   1920 x 1080
   *
   * タイヤ軸から画像下端に写る床面:
   *   200 mm
   *
   * 床面の線:
   *   線幅 2 mm
   *   線間 10 mm
   *   中心間 12 mm
   *
   * y = 1080 のとき
   * およそ200mmになる。
   */

  return (-0.153821 * y + 677.174) / (0.00144003 * y + 1.0);
}

double SquareAngleAdjustment::pixelToLateralDistance(double x, double y) const
{
  /*
   * y座標によって、
   * 横方向12mmが画像上で何pixelになるかが変化する。
   */
  const double pixelsPer12mm = 0.04893 * y + 34.414;

  if(pixelsPer12mm <= 0.0) {
    Logger::error("SquareAngleAdjustment: "
                  "invalid lateral conversion");

    return 0.0;
  }

  /*
   * x < 960 : 左
   * x = 960 : 正面
   * x > 960 : 右
   */
  return 12.0 * (x - IMAGE_CENTER_X) / pixelsPer12mm;
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
   *   → 正方形が右
   *
   * lateral < 0
   *   → 正方形が左
   *
   * atan2(
   *   横方向距離,
   *   前方距離
   * )
   *
   * でロボット正面から見た補正角度を求める。
   */
  return std::atan2(lateralDistance, forwardDistance) * SQUARE_RAD_TO_DEG;
}