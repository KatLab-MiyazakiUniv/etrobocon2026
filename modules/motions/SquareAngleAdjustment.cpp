/**
 * @file   SquareAngleAdjustment.cpp
 * @brief  カメラサーバーから取得した正方形の実距離情報から補正角度・直進距離を計算するクラス
 * @author yutaro-1214
 */

#include "SquareAngleAdjustment.h"

namespace {

  /**
   * @brief radからdegへの変換係数
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

  Logger::info("SquareAngleAdjustment: 計算を開始しました");

  // カメラサーバーとの通信用クライアントを取得する。
  SocketClient& client = robot.getCameraSocketClientInstance();

  CameraServer::SquareDetectorResponse response{};

  // 正方形の検出結果を取得する。
  const bool success = client.executeSquareDetection(request, response);

  if(!success) {
    Logger::warning("SquareAngleAdjustment: 正方形検出の通信に失敗しました");
    return result;
  }

  // 正方形が検出されなかった場合は終了する。
  if(!response.wasDetected) {
    Logger::warning("SquareAngleAdjustment: 正方形を検出できませんでした");
    return result;
  }

  // カメラサーバー側で計算済みの値を取得する。
  const double centerX = response.centerX;
  const double centerY = response.centerY;
  const double forwardDistance = response.forwardDistance;
  const double lateralDistance = response.lateralDistance;

  // 補正角度を計算する。
  const double correctionAngle = calculateCorrectionAngle(forwardDistance, lateralDistance);

  // 正方形までの直線距離を計算する。
  const double straightDistance = std::hypot(forwardDistance, lateralDistance);

  // 計算結果を格納する。
  result.wasDetected = true;
  result.centerX = centerX;
  result.centerY = centerY;
  result.forwardDistance = forwardDistance;
  result.lateralDistance = lateralDistance;
  result.correctionAngle = correctionAngle;
  result.straightDistance = straightDistance;

  // 計算結果をログに出力する。
  Logger::printfLog(Logger::INFO, "SquareAngleAdjustment: 中心座標=(%.2f, %.2f)", centerX, centerY);

  Logger::printfLog(Logger::INFO, "SquareAngleAdjustment: 前方距離=%.2f mm 横方向距離=%.2f mm",
                    forwardDistance, lateralDistance);

  Logger::printfLog(Logger::INFO, "SquareAngleAdjustment: 補正角度=%.2f deg", correctionAngle);

  Logger::printfLog(Logger::INFO, "SquareAngleAdjustment: 直線距離=%.2f mm", straightDistance);

  return result;
}

double SquareAngleAdjustment::calculateCorrectionAngle(double forwardDistance,
                                                       double lateralDistance) const
{
  // 前方距離が不正な場合は角度を計算しない。
  if(forwardDistance <= 0.0) {
    Logger::warning("SquareAngleAdjustment: 前方距離が不正なため角度を計算できませんでした");
    return 0.0;
  }

  // lateral > 0 の場合は右側、lateral < 0 の場合は左側を表す。
  return std::atan2(lateralDistance, forwardDistance) * SQUARE_RAD_TO_DEG;
}