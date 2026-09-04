/**
 * @file   RouteFollower.cpp
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#include "RouteFollower.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "AngleNormalizer.h"
#include "CameraTracking.h"
#include "DistanceCondition.h"
#include "Logger.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "Straight.h"

namespace {

  /**
   * @brief ゲートの何mm手前から正方形追従を開始するか
   */
  constexpr double SQUARE_TRACKING_BEFORE_GATE = 125.0;

  /**
   * @brief 正方形追従を行う最大距離[mm]
   *
   * ゲート125mm手前から開始し、
   * 250mm走行することで
   * ゲートの125mm先まで走行する。
   */
  constexpr double SQUARE_TRACKING_DISTANCE = 250.0;

  /**
   * @brief 回頭終了判定の許容誤差[deg]
   */
  constexpr double ROTATION_TOLERANCE = 2.0;

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
  if(route.size() < 2) {
    Logger::info(
        "RouteFollower: route is too short");

    return;
  }

  /*
   * route[0]は現在位置。
   *
   * route[1]以降を順番に走行する。
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

      rotate(
          rotationAngle);
    }

    // =====================================================
    // 2. 同じ座標
    //
    // 方向変更のみなので直進しない。
    // =====================================================

    if(from.x == to.x
       && from.y == to.y) {

      continue;
    }

    // =====================================================
    // 3. 区間距離
    // =====================================================

    const double distance =
        calculateDistance(
            from,
            to);

    if(distance <= 0.0) {

      Logger::printfLog(
          Logger::ERROR,
          "RouteFollower: invalid route distance "
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
        "RouteFollower: segment distance = %.2f mm",
        distance);

    // =====================================================
    // 4. この区間でゲートを通るか確認
    // =====================================================

    const Gate* gate =
        findGate(
            from,
            to);

    if(gate != nullptr) {

      // ===================================================
      // ゲート通過区間
      // ===================================================

      Logger::info(
          "RouteFollower: "
          "===== GATE SEGMENT -> CAMERA =====");

      runGateSegment(
          from,
          to,
          distance);

    } else {

      // ===================================================
      // 通常区間
      // ===================================================

      Logger::printfLog(
          Logger::INFO,
          "RouteFollower: "
          "NORMAL STRAIGHT %.2f mm",
          distance);

      straight(
          distance);
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
      directionToHeading(
          from);

  const double targetHeading =
      directionToHeading(
          to);

  return AngleNormalizer::normalizeAngle(
      currentHeading
      - targetHeading);
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
  // X方向
  // =====================================================

  if(from.y == to.y) {

    return std::abs(
        toNode.x
        - fromNode.x);
  }

  // =====================================================
  // Y方向
  // =====================================================

  if(from.x == to.x) {

    return std::abs(
        toNode.y
        - fromNode.y);
  }

  /*
   * RouteFollowerでは、
   * 縦または横の直線移動のみを想定する。
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
  // 距離終了条件
  // =====================================================

  auto condition =
      std::make_unique<DistanceCondition>(
          robot,
          distance);

  // =====================================================
  // 正方形検出リクエスト
  // =====================================================

  CameraServer::SquareDetectorRequest
      squareRequest {};

  /*
   * カメラ画像全体を使用する。
   */
  squareRequest.roi.x = 0;
  squareRequest.roi.y = 0;
  squareRequest.roi.width =
      CAM_MAX_WIDTH;
  squareRequest.roi.height =
      CAM_MAX_HEIGHT;

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: "
      "SquareTracking distance=%.2f "
      "speed=%.2f targetX=%d",
      distance,
      squareTrackingSpeed,
      squareTargetX);

  // =====================================================
  // CameraTracking
  // =====================================================

  CameraTracking cameraTracking(
      robot,
      std::move(condition),
      squareTrackingSpeed,
      squareTargetX,
      squareTrackingPid,
      squareRequest,
      false);

  cameraTracking.run();
}

const Gate* RouteFollower::findGate(
    const RouteState& from,
    const RouteState& to) const
{
  /*
   * 経路圧縮によって、
   *
   * from = entrance
   * to   = exit
   *
   * になるとは限らない。
   *
   * 例えば
   *
   * from ---- entrance ---- exit ---- to
   *
   * のようになる場合がある。
   *
   * そのためGatePassのentranceとexitが
   * 現在の直進区間内に存在するか確認する。
   */

  for(const Gate& gate :
      mapData.getGates()) {

    const std::vector<GatePass> passes =
        mapData.getGatePasses(
            gate.color);

    for(const GatePass& pass :
        passes) {

      bool entranceInside = false;
      bool exitInside = false;

      // ===================================================
      // X方向への移動
      // ===================================================

      if(from.y == to.y) {

        const int minX =
            std::min(
                from.x,
                to.x);

        const int maxX =
            std::max(
                from.x,
                to.x);

        /*
         * entranceが同じY座標上にあり、
         * from～toのX範囲内に存在する。
         */
        entranceInside =
            pass.entrance.y == from.y
            && pass.entrance.x >= minX
            && pass.entrance.x <= maxX;

        /*
         * exitも同じ直線区間内に存在する。
         */
        exitInside =
            pass.exit.y == from.y
            && pass.exit.x >= minX
            && pass.exit.x <= maxX;
      }

      // ===================================================
      // Y方向への移動
      // ===================================================

      else if(from.x == to.x) {

        const int minY =
            std::min(
                from.y,
                to.y);

        const int maxY =
            std::max(
                from.y,
                to.y);

        /*
         * entranceが同じX座標上にあり、
         * from～toのY範囲内に存在する。
         */
        entranceInside =
            pass.entrance.x == from.x
            && pass.entrance.y >= minY
            && pass.entrance.y <= maxY;

        /*
         * exitも同じ直線区間内に存在する。
         */
        exitInside =
            pass.exit.x == from.x
            && pass.exit.y >= minY
            && pass.exit.y <= maxY;
      }

      // ===================================================
      // ゲート通過判定
      // ===================================================

      if(entranceInside
         && exitInside) {

        Logger::printfLog(
            Logger::INFO,
            "RouteFollower: gate found "
            "entrance=(%d,%d) "
            "exit=(%d,%d)",
            pass.entrance.x,
            pass.entrance.y,
            pass.exit.x,
            pass.exit.y);

        return &gate;
      }
    }
  }

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: no gate "
      "(%d,%d) -> (%d,%d)",
      from.x,
      from.y,
      to.x,
      to.y);

  return nullptr;
}

double RouteFollower::calculateDistanceToGate(
    const RouteState& from,
    const Gate& gate) const
{
  const EtRallyMap::Node fromNode =
      map.getNode(
          from.x,
          from.y);

  // =====================================================
  // 横向きゲート
  //
  // start -------- end
  //
  // Y方向へ通過する。
  // =====================================================

  if(gate.start.y
     == gate.end.y) {

    const int centerX =
        (gate.start.x
         + gate.end.x)
        / 2;

    const int gateY =
        gate.start.y;

    const EtRallyMap::Node gateNode =
        map.getNode(
            centerX,
            gateY);

    return std::abs(
        gateNode.y
        - fromNode.y);
  }

  // =====================================================
  // 縦向きゲート
  //
  // start
  //   |
  //   |
  // end
  //
  // X方向へ通過する。
  // =====================================================

  if(gate.start.x
     == gate.end.x) {

    const int gateX =
        gate.start.x;

    const int centerY =
        (gate.start.y
         + gate.end.y)
        / 2;

    const EtRallyMap::Node gateNode =
        map.getNode(
            gateX,
            centerY);

    return std::abs(
        gateNode.x
        - fromNode.x);
  }

  Logger::error(
      "RouteFollower: invalid gate");

  return 0.0;
}

void RouteFollower::runGateSegment(
    const RouteState& from,
    const RouteState& to,
    double distance)
{
  // =====================================================
  // 1. ゲート取得
  // =====================================================

  const Gate* gate =
      findGate(
          from,
          to);

  if(gate == nullptr) {

    /*
     * 本来run()ですでにゲート判定しているので
     * ここには来ない。
     *
     * 念のため通常Straightへフォールバックする。
     */
    Logger::warning(
        "RouteFollower: "
        "gate disappeared -> Straight");

    straight(
        distance);

    return;
  }

  // =====================================================
  // 2. 現在地点からゲート中央までの距離
  // =====================================================

  const double distanceToGate =
      calculateDistanceToGate(
          from,
          *gate);

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: "
      "distance to gate = %.2f mm",
      distanceToGate);

  // =====================================================
  // 3. CameraTracking開始位置
  //
  // ゲート125mm手前
  // =====================================================

  double beforeSquareDistance =
      distanceToGate
      - SQUARE_TRACKING_BEFORE_GATE;

  /*
   * すでにゲート125mm以内にいる場合は、
   * その場からCameraTrackingする。
   */
  if(beforeSquareDistance < 0.0) {

    beforeSquareDistance = 0.0;
  }

  /*
   * 異常な距離になっている場合。
   */
  if(beforeSquareDistance > distance) {

    Logger::warning(
        "RouteFollower: "
        "gate position outside segment -> Straight");

    straight(
        distance);

    return;
  }

  // =====================================================
  // 4. ゲート125mm手前まで通常Straight
  // =====================================================

  if(beforeSquareDistance > 0.0) {

    Logger::printfLog(
        Logger::INFO,
        "RouteFollower: "
        "Straight before SquareTracking = %.2f mm",
        beforeSquareDistance);

    straight(
        beforeSquareDistance);
  }

  // =====================================================
  // 5. 残り走行可能距離
  // =====================================================

  const double availableDistance =
      distance
      - beforeSquareDistance;

  if(availableDistance <= 0.0) {

    Logger::warning(
        "RouteFollower: "
        "no distance available for SquareTracking");

    return;
  }

  // =====================================================
  // 6. 正方形追従距離
  // =====================================================

  /*
   * 基本250mm。
   *
   * 区間残距離が250mm未満なら、
   * 区間を飛び出さないよう残距離だけ走る。
   */
  const double squareDistance =
      std::min(
          SQUARE_TRACKING_DISTANCE,
          availableDistance);

  // =====================================================
  // 7. SquareTracking
  // =====================================================

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: "
      "===== SquareTracking START %.2f mm =====",
      squareDistance);

  straightWithSquareCorrection(
      squareDistance);

  Logger::info(
      "RouteFollower: "
      "===== SquareTracking FINISHED =====");

  // =====================================================
  // 8. SquareTracking後の残り距離
  // =====================================================

  const double remainingDistance =
      distance
      - beforeSquareDistance
      - squareDistance;

  if(remainingDistance > 0.0) {

    Logger::printfLog(
        Logger::INFO,
        "RouteFollower: "
        "remaining Straight = %.2f mm",
        remainingDistance);

    straight(
        remainingDistance);
  }
}