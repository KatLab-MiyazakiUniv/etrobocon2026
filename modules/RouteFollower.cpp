/**
 * @file   RouteFollower.cpp
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#include "RouteFollower.h"

#include <cmath>
#include <memory>

#include "AngleNormalizer.h"
#include "ClockUtil.h"
#include "DistanceCondition.h"
#include "Logger.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "SocketProtocol.h"
#include "SquareAngleAdjustment.h"
#include "Straight.h"
#include "SystemInfo.h"

namespace {

/**
 * @brief 回頭終了許容誤差[deg]
 */
constexpr double ROTATION_TOLERANCE = 1.0;

/**
 * @brief 正方形中心補正終了許容誤差[pixel]
 */
constexpr double SQUARE_CENTER_TOLERANCE = 5.0;

/**
 * @brief 1回目のSquare補正位置[mm]
 */
constexpr double FIRST_SQUARE_ADJUSTMENT_BEFORE_GATE = 375.0;

/**
 * @brief 2回目のSquare補正位置[mm]
 */
constexpr double SECOND_SQUARE_ADJUSTMENT_BEFORE_GATE = 125.0;

/**
 * @brief 最小回頭角度[deg]
 */
constexpr double MIN_ROTATION_ANGLE = 0.001;

/**
 * @brief 動作切替待機時間[ms]
 */
constexpr int MOTION_CHANGE_SLEEP_TIME = 400;

/**
 * @brief 動作変更前に停止する
 */
void waitBeforeMotion(
    Robot& robot)
{
  robot
      .getWheelMotorControllerInstance()
      .stopBoth();

  ClockUtil::sleep(
      MOTION_CHANGE_SLEEP_TIME);
}

}  // namespace

RouteFollower::RouteFollower(
    Robot& _robot,
    const EtRallyMap& _map,
    const MapData& _mapData,
    double _targetSpeed,
    const Pid::PidGain& _rotationPid,
    const Pid::PidGain& _rightPid,
    const Pid::PidGain& _leftPid,
    const Pid::PidGain& _straightAnglePid,
    const Pid::PidGain& _squareAnglePid)
  : robot(_robot),
    map(_map),
    mapData(_mapData),
    targetSpeed(_targetSpeed),
    rotationPid(_rotationPid),
    rightPid(_rightPid),
    leftPid(_leftPid),
    straightAnglePid(_straightAnglePid),
    squareAnglePid(_squareAnglePid),
    squareTrackingResetRequired(true)
{
  LOG_CREATE("RouteFollower");
}

void RouteFollower::run(
    const std::vector<RouteState>& route)
{
  if(route.size() < 2) {

    Logger::warning(
        "RouteFollower: route is too short");

    return;
  }

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
    // 回頭
    // =====================================================

    const double rotationAngle =
        calculateRotationAngle(
            from.direction,
            to.direction);

    if(std::abs(rotationAngle)
       > MIN_ROTATION_ANGLE) {

      Logger::printfLog(
          Logger::INFO,
          "RouteFollower: "
          "rotation required %.2f deg",
          rotationAngle);

      rotate(
          rotationAngle);
    }

    // =====================================================
    // 方向変更のみ
    // =====================================================

    if(from.x == to.x
       && from.y == to.y) {

      continue;
    }

    // =====================================================
    // 距離
    // =====================================================

    const double distance =
        calculateDistance(
            from,
            to);

    if(distance <= 0.0) {

      Logger::error(
          "RouteFollower: invalid distance");

      robot
          .getWheelMotorControllerInstance()
          .stopBoth();

      return;
    }

    Logger::printfLog(
        Logger::INFO,
        "RouteFollower: segment distance=%.2f mm",
        distance);

    // =====================================================
    // ゲート判定
    // =====================================================

    const Gate* gate =
        findGate(
            from,
            to);

    if(gate == nullptr) {

      Logger::info(
          "RouteFollower: NOT GATE SEGMENT");

      straight(
          distance);

      continue;
    }

    Logger::info(
        "RouteFollower: ===== GATE FOUND =====");

    runGateSegment(
        from,
        to,
        distance);
  }

  robot
      .getWheelMotorControllerInstance()
      .stopBoth();
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

  if(from.y == to.y) {

    return std::abs(
        toNode.x
        - fromNode.x);
  }

  if(from.x == to.x) {

    return std::abs(
        toNode.y
        - fromNode.y);
  }

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
  if(std::abs(angle)
     <= MIN_ROTATION_ANGLE) {

    return;
  }

  waitBeforeMotion(
      robot);

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

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: Rotation START %.2f",
      angle);

  rotation.run();

  robot
      .getWheelMotorControllerInstance()
      .stopBoth();

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: Rotation FINISHED %.2f",
      angle);
}

void RouteFollower::straight(
    double distance)
{
  if(distance <= 0.0) {

    return;
  }

  waitBeforeMotion(
      robot);

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

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: Straight START %.2f mm",
      distance);

  straightMotion.run();

  robot
      .getWheelMotorControllerInstance()
      .stopBoth();

  // =========================================================
  // Straightを実行したので、
  // 次回Square検出では前回位置を使用しない
  // =========================================================

  squareTrackingResetRequired =
      true;

  Logger::info(
      "RouteFollower: "
      "Straight finished -> "
      "Square tracking reset required");

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: Straight FINISHED %.2f mm",
      distance);
}

bool RouteFollower::adjustAngleWithSquare()
{
  waitBeforeMotion(
      robot);

  CameraServer::SquareDetectorRequest request {};

  request.roi.x = 0;
  request.roi.y = 0;

  request.roi.width =
      CAM_MAX_WIDTH;

  request.roi.height =
      CAM_MAX_HEIGHT;

  // =========================================================
  // Straightを挟んでいればtrue
  // =========================================================

  request.resetTracking =
      squareTrackingResetRequired;

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: "
      "Square resetTracking=%s",
      request.resetTracking
          ? "true"
          : "false");

  SquareAngleAdjustment adjustment(
      robot,
      request,
      squareAnglePid,
      SQUARE_CENTER_TOLERANCE);

  const bool success =
      adjustment.run();

  /*
   * リセット要求は今回の
   * SquareAngleAdjustmentで消費した。
   */
  squareTrackingResetRequired =
      false;

  if(!success) {

    Logger::warning(
        "RouteFollower: "
        "SquareAngleAdjustment FAILED");

    return false;
  }

  Logger::info(
      "RouteFollower: "
      "SquareAngleAdjustment SUCCESS");

  return true;
}

const Gate* RouteFollower::findGate(
    const RouteState& from,
    const RouteState& to) const
{
  const bool vertical =
      from.x == to.x;

  const bool horizontal =
      from.y == to.y;

  if(!vertical
     && !horizontal) {

    return nullptr;
  }

  for(const Gate& gate :
      mapData.getGates()) {

    const std::vector<GatePass> passes =
        mapData.getGatePasses(
            gate.color);

    for(const GatePass& pass :
        passes) {

      // ===================================================
      // Y方向
      // ===================================================

      if(vertical) {

        if(pass.entrance.x != from.x
           || pass.exit.x != from.x) {

          continue;
        }

        // Y増加
        if(to.y > from.y) {

          const bool containsGate =
              from.y <= pass.entrance.y
              && pass.entrance.y < pass.exit.y
              && pass.exit.y <= to.y;

          if(containsGate) {

            return &gate;
          }
        }

        // Y減少
        else if(to.y < from.y) {

          const bool containsGate =
              to.y <= pass.exit.y
              && pass.exit.y < pass.entrance.y
              && pass.entrance.y <= from.y;

          if(containsGate) {

            return &gate;
          }
        }
      }

      // ===================================================
      // X方向
      // ===================================================

      if(horizontal) {

        if(pass.entrance.y != from.y
           || pass.exit.y != from.y) {

          continue;
        }

        // X増加
        if(to.x > from.x) {

          const bool containsGate =
              from.x <= pass.entrance.x
              && pass.entrance.x < pass.exit.x
              && pass.exit.x <= to.x;

          if(containsGate) {

            return &gate;
          }
        }

        // X減少
        else if(to.x < from.x) {

          const bool containsGate =
              to.x <= pass.exit.x
              && pass.exit.x < pass.entrance.x
              && pass.entrance.x <= from.x;

          if(containsGate) {

            return &gate;
          }
        }
      }
    }
  }

  return nullptr;
}

bool RouteFollower::isOuterGate(
    const Gate& gate) const
{
  if(gate.start.y
     == gate.end.y) {

    const int gateY =
        gate.start.y;

    return gateY == 0
           || gateY
                  == SystemInfo::Y_GRID_NUM;
  }

  if(gate.start.x
     == gate.end.x) {

    const int gateX =
        gate.start.x;

    return gateX == 0
           || gateX
                  == SystemInfo::X_GRID_NUM;
  }

  return false;
}

double RouteFollower::calculateDistanceToGate(
    const RouteState& from,
    const Gate& gate) const
{
  const EtRallyMap::Node fromNode =
      map.getNode(
          from.x,
          from.y);

  // =========================================================
  // 横向きゲート
  // =========================================================

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

  // =========================================================
  // 縦向きゲート
  // =========================================================

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
  const Gate* gate =
      findGate(
          from,
          to);

  if(gate == nullptr) {

    straight(
        distance);

    return;
  }

  // =========================================================
  // 外周ゲート
  // =========================================================

  if(isOuterGate(
         *gate)) {

    straight(
        distance);

    return;
  }

  // =========================================================
  // 内側ゲート
  // =========================================================

  const double distanceToGate =
      calculateDistanceToGate(
          from,
          *gate);

  Logger::printfLog(
      Logger::INFO,
      "RouteFollower: "
      "distanceToGate=%.2f mm",
      distanceToGate);

  if(distanceToGate <= 0.0) {

    straight(
        distance);

    return;
  }

  double traveledDistance =
      0.0;

  // =========================================================
  // 375mm手前
  // =========================================================

  const double firstAdjustmentPosition =
      distanceToGate
      - FIRST_SQUARE_ADJUSTMENT_BEFORE_GATE;

  if(firstAdjustmentPosition
     > traveledDistance) {

    const double firstStraightDistance =
        firstAdjustmentPosition
        - traveledDistance;

    straight(
        firstStraightDistance);

    traveledDistance +=
        firstStraightDistance;
  }

  // =========================================================
  // 1回目Square補正
  // =========================================================

  if(distanceToGate
     > SECOND_SQUARE_ADJUSTMENT_BEFORE_GATE) {

    Logger::info(
        "RouteFollower: "
        "FIRST SquareAngleAdjustment");

    if(!adjustAngleWithSquare()) {

      Logger::warning(
          "RouteFollower: "
          "FIRST square adjustment failed");
    }
  }

  // =========================================================
  // 125mm手前
  // =========================================================

  const double secondAdjustmentPosition =
      distanceToGate
      - SECOND_SQUARE_ADJUSTMENT_BEFORE_GATE;

  if(secondAdjustmentPosition
     > traveledDistance) {

    const double secondStraightDistance =
        secondAdjustmentPosition
        - traveledDistance;

    /*
     * ここでStraightを実行するので、
     * squareTrackingResetRequiredがtrueになる。
     *
     * したがって次の2回目補正では
     * SquareDetectorの履歴がリセットされる。
     */
    straight(
        secondStraightDistance);

    traveledDistance +=
        secondStraightDistance;
  }

  // =========================================================
  // 2回目Square補正
  // =========================================================

  if(distanceToGate
     >= SECOND_SQUARE_ADJUSTMENT_BEFORE_GATE) {

    Logger::info(
        "RouteFollower: "
        "SECOND SquareAngleAdjustment");

    if(!adjustAngleWithSquare()) {

      Logger::warning(
          "RouteFollower: "
          "SECOND square adjustment failed");
    }
  }

  // =========================================================
  // 残距離
  // =========================================================

  const double remainingDistance =
      distance
      - traveledDistance;

  if(remainingDistance > 0.0) {

    straight(
        remainingDistance);
  }
}