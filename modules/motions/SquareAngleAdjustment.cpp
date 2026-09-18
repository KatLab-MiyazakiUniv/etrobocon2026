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
   * @brief ロボットの走行距離基準点から
   *        カメラ基準位置までの前後方向距離[mm]
   *
   * カメラはロボットの走行距離基準点より前方に取り付けられているため、
   * カメラから正方形までの距離だけを使うと、
   * その分だけ走行距離が短くなる。
   *
   * 実機の寸法に合わせて調整する。
   */
  constexpr double ROBOT_TO_CAMERA_OFFSET = 100.0;

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

  Logger::info("SquareAngleAdjustment: "
               "calculate CALLED");

  // =====================================================
  // 1. SocketClient取得
  // =====================================================

  SocketClient& client
      = robot.getCameraSocketClientInstance();

  CameraServer::SquareDetectorResponse response{};

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

  /**
   * カメラ基準の前方距離
   */
  const double cameraForwardDistance
      = response.forwardDistance;

  /**
   * 横方向距離
   *
   * 横方向については、
   * カメラとロボット中心の横位置が一致している前提で
   * そのまま使用する。
   */
  const double lateralDistance
      = response.lateralDistance;

  // =====================================================
  // 5. ロボット基準の前方距離に補正
  //
  // カメラがロボット基準点より前にあるため、
  // カメラまでのオフセットを加える。
  // =====================================================

  const double forwardDistance
      = cameraForwardDistance
        + ROBOT_TO_CAMERA_OFFSET;

  // =====================================================
  // 6. 補正角度
  //
  // ロボット基準点から見た
  // 正方形の方向を計算する。
  // =====================================================

  const double correctionAngle
      = calculateCorrectionAngle(
          forwardDistance,
          lateralDistance);

  // =====================================================
  // 7. 正方形までの直線距離
  //
  // ロボット基準の前方距離と
  // 横方向距離から
  // 正方形までの直線距離を求める。
  // =====================================================

  const double straightDistance
      = std::hypot(
          forwardDistance,
          lateralDistance);

  // =====================================================
  // 8. 結果格納
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
  // 9. ログ
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
      "cameraForward=%.2f mm",
      cameraForwardDistance);

  Logger::printfLog(
      Logger::INFO,
      "SquareAngleAdjustment: "
      "robotToCameraOffset=%.2f mm",
      ROBOT_TO_CAMERA_OFFSET);

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
      "straightDistance=%.2f mm",
      straightDistance);

  return result;
}

double SquareAngleAdjustment::calculateCorrectionAngle(
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
  return std::atan2(
             lateralDistance,
             forwardDistance)
         * SQUARE_RAD_TO_DEG;
}