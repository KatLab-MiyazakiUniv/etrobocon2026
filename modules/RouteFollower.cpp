/**
 * @file   RouteFollower.cpp
 * @brief  経路探索結果に従って正方形を追従しながら走行するクラス
 * @author yutaro-1214
 */

#include "RouteFollower.h"

#include <cmath>
#include <memory>

#include "AngleNormalizer.h"
#include "CameraTracking.h"
#include "ClockUtil.h"
#include "DistanceCondition.h"
#include "Logger.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "SocketClient.h"
#include "Straight.h"

namespace {

  /**
   * @brief 回頭終了判定の許容誤差[deg]
   */
  constexpr double ROTATION_TOLERANCE = 2.0;

  /**
   * @brief 最初にカメラが使用可能になるまで待つ時間[ms]
   */
  constexpr int CAMERA_WARMUP_TIME = 3000;

}  // namespace

RouteFollower::RouteFollower(
    Robot& _robot,
    const EtRallyMap& _map,
    const MapData& _mapData,
    double _targetSpeed,
    double _squareTrackingSpeed,
    int _squareTargetX,
    const Pid::PidGain& _rotationPid,
    const Pid::PidGain& _rightPid,
    const Pid::PidGain& _leftPid,
    const Pid::PidGain& _straightAnglePid,
    const Pid::PidGain& _squareTrackingPid)
  : robot(_robot),
    map(_map),
    mapData(_mapData),
    targetSpeed(_targetSpeed),
    squareTrackingSpeed(_squareTrackingSpeed),
    squareTargetX(_squareTargetX),
    rotationPid(_rotationPid),
    rightPid(_rightPid),
    leftPid(_leftPid),
    straightAnglePid(_straightAnglePid),
    squareTrackingPid(_squareTrackingPid)
{
  LOG_CREATE("RouteFollower");
}

void RouteFollower::run(
    const std::vector<RouteState>& route)
{
  /*
   * デフォルトで全区間正方形走行を使用する。
   */
  run(route, true);
}

void RouteFollower::run(
    const std::vector<RouteState>& route,
    bool useSquareCorrection)
{
  if(route.size() < 2) {
    Logger::info(
        "RouteFollower: route is too short");

    return;
  }

  /*
   * プログラム実行中、
   * 最初の正方形走行前だけカメラ待機を行う。
   *
   * run()はゲートごとに複数回呼ばれるため、
   * staticにして一度だけ実行する。
   */
  static bool cameraWarmupDone = false;

  /*
   * route[0]は現在位置。
   * route[1]から順番に走行する。
   */
  for(std::size_t i = 1;
      i < route.size();
      ++i) {

    const RouteState& from =
        route[i - 1];

    const RouteState& to =
        route[i];

    Logger::printfLog(
        Logger::INFO,
        "RouteFollower[%d]: (%d,%d) -> (%d,%d)",
        static_cast<int>(i),
        from.x,
        from.y,
        to.x,
        to.y);

    // =====================================================
    // 1. 回頭
    // =====================================================

    const double rotationAngle =
        calculateRotationAngle(
            from.direction,
            to.direction);

    if(std::abs(rotationAngle) > 0.001) {

      Logger::printfLog(
          Logger::INFO,
          "RouteFollower: Rotate %.2f deg",
          rotationAngle);

      rotate(rotationAngle);
    }

    // =====================================================
    // 2. 座標が同じ場合
    //
    // 方向変更だけなので直進しない。
    // =====================================================

    if(from.x == to.x
       && from.y == to.y) {

      continue;
    }

    // =====================================================
    // 3. 直進距離を計算
    // =====================================================

    const double distance =
        calculateDistance(
            from,
            to);

    if(distance <= 0.0) {

      Logger::printfLog(
          Logger::ERROR,
          "RouteFollower: invalid distance "
          "(%d,%d) -> (%d,%d)",
          from.x,
          from.y,
          to.x,
          to.y);

      robot
          .getWheelMotorControllerInstance()
          .stopBoth();

      return;
    }

    Logger::printfLog(
        Logger::INFO,
        "RouteFollower: distance = %.2f mm",
        distance);

    // =====================================================
    // 4. 正方形走行
    // =====================================================

    if(useSquareCorrection) {

      /*
       * 最初の正方形走行前だけ、
       * カメラが起動する時間を確保する。
       */
      if(!cameraWarmupDone) {

        Logger::info(
            "RouteFollower: "
            "========================================");

        Logger::printfLog(
            Logger::INFO,
            "RouteFollower: wait %d ms for camera warmup",
            CAMERA_WARMUP_TIME);

        /*
         * 念のためモータを止めてから待機する。
         */
        robot
            .getWheelMotorControllerInstance()
            .stopBoth();

        ClockUtil::sleep(
            CAMERA_WARMUP_TIME);

        Logger::info(
            "RouteFollower: camera warmup wait finished");

        Logger::info(
            "RouteFollower: "
            "========================================");

        cameraWarmupDone = true;
      }

      Logger::printfLog(
          Logger::INFO,
          "RouteFollower: "
          "SQUARE TRACKING start distance=%.2f mm",
          distance);

      straightWithSquareCorrection(
          distance);

      Logger::info(
          "RouteFollower: "
          "SQUARE TRACKING finished");

    } else {

      // ===================================================
      // falseが指定された場合のみ通常Straight
      // ===================================================

      Logger::printfLog(
          Logger::INFO,
          "RouteFollower: "
          "normal Straight distance=%.2f mm",
          distance);

      straight(distance);
    }
  }
}

double RouteFollower::directionToHeading(
    Direction direction) const
{
  switch(direction) {

    case Direction::RIGHT:
      return 0.0;

    case Direction::UP:
      return 90.0;

    case Direction::LEFT:
      return 180.0;

    case Direction::DOWN:
      return -90.0;
  }

  return 0.0;
}

double RouteFollower::calculateRotationAngle(
    Direction from,
    Direction to) const
{
  const double currentHeading =
      directionToHeading(from);

  const double targetHeading =
      directionToHeading(to);

  return AngleNormalizer::normalizeAngle(
      currentHeading - targetHeading);
}

double RouteFollower::calculateDistance(
    const RouteState& from,
    const RouteState& to) const
{
  const EtRallyMap::Node fromNode =
      map.getNode(
          from.x,
          from.y);

  const EtRallyMap::Node toNode =
      map.getNode(
          to.x,
          to.y);

  // =====================================================
  // X方向への移動
  // =====================================================

  if(from.y == to.y) {

    return std::abs(
        toNode.x
        - fromNode.x);
  }

  // =====================================================
  // Y方向への移動
  // =====================================================

  if(from.x == to.x) {

    return std::abs(
        toNode.y
        - fromNode.y);
  }

  /*
   * 圧縮済み経路は
   * 縦または横のみを想定する。
   */
  Logger::printfLog(
      Logger::ERROR,
      "RouteFollower: diagonal route "
      "(%d,%d) -> (%d,%d)",
      from.x,
      from.y,
      to.x,
      to.y);

  return 0.0;
}

void RouteFollower::rotate(
    double angle)
{
  auto condition =
      std::make_unique<RelativeAngleCondition>(
          robot,
          angle,
          ROTATION_TOLERANCE);

  RelativeRotation rotation(
      robot,
      std::move(condition),
      rotationPid,
      angle);

  rotation.run();
}

void RouteFollower::straight(
    double distance)
{
  if(distance <= 0.0) {
    return;
  }

  auto condition =
      std::make_unique<DistanceCondition>(
          robot,
          distance);

  Straight straightMotion(
      robot,
      std::move(condition),
      targetSpeed,
      rightPid,
      leftPid,
      straightAnglePid,
      true);

  straightMotion.run();
}

void RouteFollower::straightWithSquareCorrection(
    double distance)
{
  if(distance <= 0.0) {
    return;
  }

  // =====================================================
  // 走行終了条件
  // =====================================================

  auto condition =
      std::make_unique<DistanceCondition>(
          robot,
          distance);

  // =====================================================
  // 正方形検出リクエスト
  // =====================================================

  CameraServer::SquareDetectorRequest squareRequest {};

  /*
   * 現在はカメラ画像全体を検出対象にする。
   */
  squareRequest.roi.x = 0;
  squareRequest.roi.y = 0;
  squareRequest.roi.width = CAM_MAX_WIDTH;
  squareRequest.roi.height = CAM_MAX_HEIGHT;

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: create Square CameraTracking "
      "distance=%.2f speed=%.2f targetX=%d",
      distance,
      squareTrackingSpeed,
      squareTargetX);

  // =====================================================
  // CameraTracking生成
  // =====================================================

  CameraTracking cameraTracking(
      robot,
      std::move(condition),
      squareTrackingSpeed,
      squareTargetX,
      squareTrackingPid,
      squareRequest,
      false);

  // =====================================================
  // 正方形追従開始
  // =====================================================

  cameraTracking.run();
}

bool RouteFollower::detectSquare()
{
  SocketClient& client =
      robot.getCameraSocketClientInstance();

  CameraServer::SquareDetectorRequest request {};
  CameraServer::SquareDetectorResponse response {};

  // =====================================================
  // ROI
  // =====================================================

  request.roi.x = 0;
  request.roi.y = 0;
  request.roi.width = CAM_MAX_WIDTH;
  request.roi.height = CAM_MAX_HEIGHT;

  Logger::info(
      "RouteFollower: square detection request");

  // =====================================================
  // 正方形検出
  // =====================================================

  const bool success =
      client.executeSquareDetection(
          request,
          response);

  // =====================================================
  // 通信失敗
  // =====================================================

  if(!success) {

    Logger::error(
        "RouteFollower: "
        "square detection communication failed");

    return false;
  }

  Logger::info(
      "RouteFollower: "
      "square detection communication success");

  // =====================================================
  // 正方形未検出
  // =====================================================

  if(!response.wasDetected) {

    Logger::warning(
        "RouteFollower: "
        "SQUARE NOT DETECTED");

    return false;
  }

  // =====================================================
  // 正方形検出成功
  // =====================================================

  Logger::info(
      "RouteFollower: "
      "===== SQUARE DETECTED =====");

  double sumX = 0.0;
  double sumY = 0.0;

  for(int i = 0;
      i < CameraServer::SQUARE_CORNER_COUNT;
      ++i) {

    Logger::printfLog(
        Logger::INFO,
        "RouteFollower: corner[%d] = (%d,%d)",
        i,
        response.corners[i].x,
        response.corners[i].y);

    sumX += response.corners[i].x;
    sumY += response.corners[i].y;
  }

  const double centerX =
      sumX
      / CameraServer::SQUARE_CORNER_COUNT;

  const double centerY =
      sumY
      / CameraServer::SQUARE_CORNER_COUNT;

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: "
      "square center=(%.2f,%.2f)",
      centerX,
      centerY);

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: "
      "targetX=%d errorX=%.2f",
      squareTargetX,
      centerX
          - static_cast<double>(
                squareTargetX));

  return true;
}

const Gate* RouteFollower::findGate(
    const RouteState& from,
    const RouteState& to) const
{
  /*
   * 現在は全区間を正方形走行するため、
   * 基本的には使用しない。
   *
   * RouteFollower.hとの互換性のため残している。
   */

  for(const Gate& gate :
      mapData.getGates()) {

    const std::vector<GatePass> passes =
        mapData.getGatePasses(
            gate.color);

    for(const GatePass& pass :
        passes) {

      const bool entranceMatches =
          pass.entrance.x == from.x
          && pass.entrance.y == from.y;

      const bool exitMatches =
          pass.exit.x == to.x
          && pass.exit.y == to.y;

      if(entranceMatches
         && exitMatches) {

        return &gate;
      }
    }
  }

  return nullptr;
}

bool RouteFollower::isOuterGate(
    const Gate& gate) const
{
  /*
   * 全区間正方形走行では
   * 外周判定を使用しない。
   *
   * 既存ヘッダとの互換性のため残す。
   */

  (void)gate;

  return false;
}

double RouteFollower::calculateDistanceToGate(
    const RouteState& from,
    const Gate& gate) const
{
  /*
   * 全区間正方形走行では
   * ゲートまでの距離を使用しない。
   *
   * 既存ヘッダとの互換性のため残す。
   */

  (void)from;
  (void)gate;

  return 0.0;
}

void RouteFollower::runGateSegment(
    const RouteState& from,
    const RouteState& to,
    double distance)
{
  /*
   * 全区間正方形走行に変更したため、
   * ゲート区間も通常区間も同じ処理を行う。
   */

  (void)from;
  (void)to;

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: gate segment "
      "-> SquareTracking %.2f mm",
      distance);

  straightWithSquareCorrection(
      distance);
}