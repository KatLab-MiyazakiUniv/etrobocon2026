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
   *
   * カメラがロボット基準位置より前方に100mmあるものとして、
   * カメラ基準で取得した前方距離に100mmを加える。
   *
   * 例：
   *
   * カメラから正方形まで200mmの場合、
   *
   *   ロボット基準位置
   *        ↓
   *   [Robot] ----100mm---- [Camera] ----200mm---- [Square]
   *
   * ロボット基準では正方形まで300mmとなる。
   */
  constexpr double CAMERA_TO_ROBOT_OFFSET = 0;

  /**
   * @brief ロボット基準で使用可能な最小前方距離[mm]
   *
   * 負の距離や極端に近い異常値を除外する。
   */
  constexpr double MIN_FORWARD_DISTANCE = 0.0;

  /**
   * @brief ロボット基準で使用可能な最大前方距離[mm]
   *
   * 正方形補正で現実的に使用する最大距離。
   * 必要に応じて実機に合わせて変更する。
   */
  constexpr double MAX_FORWARD_DISTANCE = 700.0;

  /**
   * @brief 使用可能な横方向距離の最大絶対値[mm]
   *
   * 左右300mmを超える値は、
   * 誤検出または座標変換異常の可能性が高いとして除外する。
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

  // =========================================================
  // カメラサーバーとの通信
  // =========================================================

  SocketClient& client = robot.getCameraSocketClientInstance();

  CameraServer::SquareDetectorResponse response{};

  // 正方形の検出結果を取得する。
  const bool success = client.executeSquareDetection(request, response);

  // =========================================================
  // 通信失敗
  // =========================================================

  if(!success) {
    Logger::warning("SquareAngleAdjustment: "
                    "正方形検出の通信に失敗しました");

    return result;
  }

  // =========================================================
  // 正方形未検出
  // =========================================================

  if(!response.wasDetected) {
    Logger::warning("SquareAngleAdjustment: "
                    "正方形を検出できませんでした");

    return result;
  }

  // =========================================================
  // カメラサーバー側で計算済みの値を取得
  // =========================================================

  const double centerX = response.centerX;

  const double centerY = response.centerY;

  // カメラ基準の前方距離
  const double cameraForwardDistance = response.forwardDistance;

  // 横方向距離
  const double lateralDistance = response.lateralDistance;

  // =========================================================
  // カメラ基準 → ロボット基準へ変換
  // =========================================================
  //
  // カメラがロボット基準位置より100mm前にあるため、
  // カメラから正方形までの前方距離に100mm加える。
  //
  // 以前は
  //
  //   hypot(forward, lateral) + 100
  //
  // としていたが、
  // 今回は座標そのものをロボット基準へ変換してから
  // 距離と角度を計算する。
  // =========================================================

  const double robotForwardDistance = cameraForwardDistance + CAMERA_TO_ROBOT_OFFSET;

  // =========================================================
  // ロボット基準での有効範囲チェック
  // =========================================================
  //
  // 「用紙の範囲内か」ではなく、
  // ロボットから見て現実的な距離かどうかを確認する。
  // =========================================================

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

  if(std::abs(lateralDistance) > MAX_LATERAL_DISTANCE) {
    Logger::printfLog(Logger::WARNING,
                      "SquareAngleAdjustment: "
                      "横方向距離が範囲外です "
                      "%.2f mm",
                      lateralDistance);

    return result;
  }

  // =========================================================
  // 補正角度を計算
  // =========================================================
  //
  // ロボット基準に変換した前方距離と
  // 横方向距離から正方形方向への角度を計算する。
  //
  // lateral > 0 : 右
  // lateral < 0 : 左
  // =========================================================

  const double correctionAngle = calculateCorrectionAngle(robotForwardDistance, lateralDistance);

  // =========================================================
  // ロボットから正方形までの直線距離
  // =========================================================

  const double straightDistance = std::hypot(robotForwardDistance, lateralDistance);

  // =========================================================
  // 計算結果を格納
  // =========================================================

  result.wasDetected = true;

  result.centerX = centerX;

  result.centerY = centerY;

  // Result側にはロボット基準の距離を格納する。
  result.forwardDistance = robotForwardDistance;

  result.lateralDistance = lateralDistance;

  result.correctionAngle = correctionAngle;

  result.straightDistance = straightDistance;

  // =========================================================
  // ログ
  // =========================================================

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
  // =========================================================
  // 前方距離チェック
  // =========================================================

  if(forwardDistance <= 0.0) {
    Logger::warning("SquareAngleAdjustment: "
                    "前方距離が不正なため"
                    "角度を計算できませんでした");

    return 0.0;
  }

  // =========================================================
  // 補正角度
  // =========================================================
  //
  // lateral > 0 : 右側
  // lateral < 0 : 左側
  //
  // atan2(横距離, 前方距離)
  // =========================================================

  return std::atan2(lateralDistance, forwardDistance) * SQUARE_RAD_TO_DEG;
}