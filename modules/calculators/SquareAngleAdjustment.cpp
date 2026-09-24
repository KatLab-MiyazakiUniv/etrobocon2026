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

  /**
   * @brief カメラ位置からロボット基準位置までの前後方向オフセット[mm]
   */
  constexpr double CAMERA_TO_ROBOT_OFFSET = 0.0;

  /**
   * @brief 使用可能な最小前方距離[mm]
   */
  constexpr double MIN_FORWARD_DISTANCE = 0.0;

  /**
   * @brief 使用可能な最大前方距離[mm]
   */
  constexpr double MAX_FORWARD_DISTANCE = 700.0;

  /**
   * @brief 使用可能な横方向距離の最大絶対値[mm]
   */
  constexpr double MAX_LATERAL_DISTANCE = 300.0;

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

  SocketClient& client = robot.getCameraSocketClientInstance();

  CameraServer::SquareDetectorResponse response{};

  // カメラサーバーから正方形の検出結果を取得する
  const bool success = client.executeSquareDetection(request, response);

  if(!success) {
    Logger::warning("SquareAngleAdjustment: "
                    "正方形検出の通信に失敗しました");

    return result;
  }

  // 正方形を検出できなかった場合は計算を終了する
  if(!response.wasDetected) {
    Logger::warning("SquareAngleAdjustment: "
                    "正方形を検出できませんでした");

    return result;
  }

  // カメラサーバーで計算された正方形の位置情報を取得する
  const double centerX = response.centerX;
  const double centerY = response.centerY;
  const double cameraForwardDistance = response.forwardDistance;
  const double lateralDistance = response.lateralDistance;

  // カメラ基準の前方距離をロボット基準の前方距離へ変換する
  const double robotForwardDistance = cameraForwardDistance + CAMERA_TO_ROBOT_OFFSET;

  // 前方距離が使用可能な範囲内か確認する
  if(robotForwardDistance <= MIN_FORWARD_DISTANCE) {
    Logger::printfLog(Logger::WARNING,
                      "SquareAngleAdjustment: "
                      "前方距離が範囲外です "
                      "%.2f mm",
                      robotForwardDistance);

    return result;
  }

  if(robotForwardDistance > MAX_FORWARD_DISTANCE) {
    Logger::printfLog(Logger::WARNING,
                      "SquareAngleAdjustment: "
                      "前方距離が遠すぎます "
                      "%.2f mm",
                      robotForwardDistance);

    return result;
  }

  // 横方向距離が使用可能な範囲内か確認する
  if(std::abs(lateralDistance) > MAX_LATERAL_DISTANCE) {
    Logger::printfLog(Logger::WARNING,
                      "SquareAngleAdjustment: "
                      "横方向距離が範囲外です "
                      "%.2f mm",
                      lateralDistance);

    return result;
  }

  // ロボットから見た正方形方向への補正角度を計算する
  const double correctionAngle = calculateCorrectionAngle(robotForwardDistance, lateralDistance);

  // ロボットから正方形までの直線距離を計算する
  const double straightDistance = std::hypot(robotForwardDistance, lateralDistance);

  // 計算結果を格納する
  result.wasDetected = true;
  result.centerX = centerX;
  result.centerY = centerY;
  result.forwardDistance = robotForwardDistance;
  result.lateralDistance = lateralDistance;
  result.correctionAngle = correctionAngle;
  result.straightDistance = straightDistance;

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "中心座標=(%.2f, %.2f)",
                    centerX, centerY);

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "カメラ基準前方距離=%.2f mm",
                    cameraForwardDistance);

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "カメラ→ロボット基準オフセット=%.2f mm",
                    CAMERA_TO_ROBOT_OFFSET);

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "ロボット基準前方距離=%.2f mm",
                    robotForwardDistance);

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "横方向距離=%.2f mm",
                    lateralDistance);

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "補正角度=%.2f deg",
                    correctionAngle);

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "ロボットから正方形までの距離=%.2f mm",
                    straightDistance);

  return result;
}

double SquareAngleAdjustment::calculateCorrectionAngle(double forwardDistance,
                                                       double lateralDistance) const
{
  // 前方距離が不正な場合は角度を計算しない
  if(forwardDistance <= 0.0) {
    Logger::warning("SquareAngleAdjustment: "
                    "前方距離が不正なため"
                    "角度を計算できませんでした");

    return 0.0;
  }

  // 前方距離と横方向距離から正方形方向への補正角度を計算する
  return std::atan2(lateralDistance, forwardDistance) * SQUARE_RAD_TO_DEG;
}