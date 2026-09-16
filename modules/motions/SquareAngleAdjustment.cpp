/**
 * @file   SquareAngleAdjustment.cpp
 * @brief  カメラサーバーから取得した正方形の実距離情報から
 *         補正角度・直進距離を計算するクラス
 * @author yutaro-1214
 */

#include "SquareAngleAdjustment.h"

#include <cmath>

#include "Logger.h"
#include "SocketClient.h"

namespace {

  /**
   * @brief radからdegへの変換係数
   */
  constexpr double SQUARE_RAD_TO_DEG
      = 180.0 / 3.14159265358979323846;

  /**
   * @brief 正方形まで進む距離の補正係数
   *
   * 計算された距離より少し手前で停止する。
   *
   * 1.00 : 計算距離そのまま
   * 0.90 : 計算距離の90%
   */
  constexpr double DISTANCE_SCALE
      = 0.90;

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
  // 2. 正方形検出要求
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

  // =====================================================
  // 3. 正方形未検出
  // =====================================================

  if(!response.wasDetected) {

    Logger::warning(
        "SquareAngleAdjustment: "
        "square not detected");

    return result;
  }

  // =====================================================
  // 4. カメラサーバー側で計算済みの情報を取得
  //
  // 画像処理・ホモグラフィ変換は
  // カメラサーバー側で実施する。
  // =====================================================

  const double centerX
      = response.centerX;

  const double centerY
      = response.centerY;

  const double forwardDistance
      = response.forwardDistance;

  const double lateralDistance
      = response.lateralDistance;

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

  const double calculatedDistance
      = std::hypot(
          forwardDistance,
          lateralDistance);

  /*
   * 少し手前で停止させるため、
   * 補正係数を掛ける。
   */
  const double straightDistance
      = calculatedDistance
        * DISTANCE_SCALE;

  // =====================================================
  // 7. 結果格納
  // =====================================================

  result.wasDetected
      = true;

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
      "pixel=(%.2f, %.2f)",
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
      "angle=%.2f deg",
      correctionAngle);

  Logger::printfLog(
      Logger::INFO,
      "SquareAngleAdjustment: "
      "calculatedDistance=%.2f mm "
      "straightDistance=%.2f mm",
      calculatedDistance,
      straightDistance);

  return result;
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

  /*
   * lateral > 0
   *   -> 正方形が右側
   *
   * lateral < 0
   *   -> 正方形が左側
   */
  return
      std::atan2(
          lateralDistance,
          forwardDistance)
      * SQUARE_RAD_TO_DEG;
}