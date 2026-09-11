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
#include "Straight.h"
#include "SystemInfo.h"

namespace {

  /**
   * @brief 回頭終了許容誤差[deg]
   */
  constexpr double ROTATION_TOLERANCE = 2.0;

  /**
   * @brief QR①とQR②の実距離[mm]
   *
   * QR①
   *  |
   *  | 125mm
   *  |
   * ゲート
   *  |
   *  | 125mm
   *  |
   * QR②
   */
  constexpr double QR_DISTANCE = 250.0;

  /**
   * @brief QRからゲート中央までの距離[mm]
   */
  constexpr double QR_TO_GATE_DISTANCE = QR_DISTANCE / 2.0;

  /**
   * @brief 動作切り替え時の停止時間[ms]
   */
  constexpr int MOTION_SWITCH_WAIT = 200;

}  // namespace

RouteFollower::RouteFollower(Robot& _robot, const EtRallyMap& _map, const MapData& _mapData,
                             double _targetSpeed, const Pid::PidGain& _rotationPid,
                             const Pid::PidGain& _rightPid, const Pid::PidGain& _leftPid,
                             const Pid::PidGain& _straightAnglePid)
  : robot(_robot),
    map(_map),
    mapData(_mapData),
    targetSpeed(_targetSpeed),
    rotationPid(_rotationPid),
    rightPid(_rightPid),
    leftPid(_leftPid),
    straightAnglePid(_straightAnglePid)
{
  LOG_CREATE("RouteFollower");
}

void RouteFollower::run(const std::vector<RouteState>& route)
{
  if(route.size() < 2) {
    Logger::warning("RouteFollower: route size < 2");

    return;
  }

  /*
   * route[0]は現在位置。
   * route[1]から順番に走行する。
   */
  for(std::size_t i = 1; i < route.size(); ++i) {
    const RouteState& from = route[i - 1];

    const RouteState& to = route[i];

    Logger::printfLog(Logger::INFO,
                      "RouteFollower[%d]: "
                      "(%d,%d) -> (%d,%d)",
                      static_cast<int>(i), from.x, from.y, to.x, to.y);

    // =====================================================
    // 1. 必要なら回頭
    // =====================================================

    const double rotationAngle = calculateRotationAngle(from.direction, to.direction);

    if(std::abs(rotationAngle) > ROTATION_TOLERANCE) {
      Logger::printfLog(Logger::INFO,
                        "RouteFollower: "
                        "rotation required %.2f deg",
                        rotationAngle);

      rotate(rotationAngle);
    }

    // =====================================================
    // 2. 座標が同じ場合
    //
    // 最終方向変更など、
    // 回頭だけのRouteState。
    // =====================================================

    if(from.x == to.x && from.y == to.y) {
      continue;
    }

    // =====================================================
    // 3. 区間距離
    // =====================================================

    const double distance = calculateDistance(from, to);

    if(distance <= 0.0) {
      Logger::error("RouteFollower: invalid segment distance");

      robot.getWheelMotorControllerInstance().stopBoth();

      return;
    }

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "segment distance=%.2f mm",
                      distance);

    // =====================================================
    // 4. ゲート区間判定
    // =====================================================

    const Gate* gate = findGate(from, to);

    if(gate != nullptr) {
      Logger::info("RouteFollower: ===== GATE FOUND =====");

      runGateSegment(from, to, distance);

      continue;
    }

    // =====================================================
    // 5. 通常区間
    // =====================================================

    straight(distance);
  }
}

double RouteFollower::directionToHeading(Direction direction) const
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

double RouteFollower::calculateRotationAngle(Direction from, Direction to) const
{
  const double currentHeading = directionToHeading(from);

  const double targetHeading = directionToHeading(to);

  /*
   * 現在のRouteFollowerの座標系に合わせる。
   */
  return AngleNormalizer::normalizeAngle(currentHeading - targetHeading);
}

double RouteFollower::calculateDistance(const RouteState& from, const RouteState& to) const
{
  const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);

  const EtRallyMap::Node toNode = map.getNode(to.x, to.y);

  // X方向
  if(from.y == to.y) {
    return std::abs(toNode.x - fromNode.x);
  }

  // Y方向
  if(from.x == to.x) {
    return std::abs(toNode.y - fromNode.y);
  }

  Logger::printfLog(Logger::ERROR,
                    "RouteFollower: diagonal route "
                    "(%d,%d) -> (%d,%d)",
                    from.x, from.y, to.x, to.y);

  return 0.0;
}

void RouteFollower::rotate(double angle)
{
  if(std::abs(angle) <= ROTATION_TOLERANCE) {
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "rotation skipped %.2f deg",
                      angle);

    return;
  }

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  auto condition = std::make_unique<RelativeAngleCondition>(robot, angle, ROTATION_TOLERANCE);

  RelativeRotation rotation(robot, std::move(condition), rotationPid, angle);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Rotation START %.2f deg",
                    angle);

  rotation.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Rotation FINISHED %.2f deg",
                    angle);
}

void RouteFollower::straight(double distance)
{
  if(distance <= 0.0) {
    Logger::printfLog(Logger::WARNING,
                      "RouteFollower: "
                      "Straight skipped %.2f mm",
                      distance);

    return;
  }

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  auto condition = std::make_unique<DistanceCondition>(robot, distance);

  Straight straightMotion(robot, std::move(condition), targetSpeed, rightPid, leftPid,
                          straightAnglePid, true);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Straight START %.2f mm",
                    distance);

  straightMotion.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Straight FINISHED %.2f mm",
                    distance);
}

bool RouteFollower::detectSquare(SquareAngleAdjustment::Result& result)
{
  // =====================================================
  // 1. 停止
  // =====================================================

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  // =====================================================
  // 2. 検出リクエスト
  // =====================================================

  CameraServer::SquareDetectorRequest request{};

  /*
   * 画像全体を使用する。
   *
   * 1920 x 1080
   */
  request.roi.x = 0;
  request.roi.y = 0;
  request.roi.width = CAM_MAX_WIDTH;
  request.roi.height = CAM_MAX_HEIGHT;

  // =====================================================
  // 3. 正方形検出
  // =====================================================

  SquareAngleAdjustment adjustment(robot);

  result = adjustment.calculate(request);

  if(!result.wasDetected) {
    Logger::warning("RouteFollower: "
                    "Square detection FAILED");

    return false;
  }

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Square center=(%.2f, %.2f)",
                    result.centerX, result.centerY);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Square position "
                    "forward=%.2f "
                    "lateral=%.2f",
                    result.forwardDistance, result.lateralDistance);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "Square angle=%.2f "
                    "distance=%.2f",
                    result.correctionAngle, result.straightDistance);

  return true;
}

void RouteFollower::runGateSegment(const RouteState& from, const RouteState& to, double distance)
{
  const Gate* gate = findGate(from, to);

  if(gate == nullptr) {
    straight(distance);

    return;
  }

  // =====================================================
  // 外周ゲート
  // =====================================================

  if(isOuterGate(*gate)) {
    Logger::info("RouteFollower: OUTER GATE");

    straight(distance);

    return;
  }

  // =====================================================
  // 内側ゲート
  // =====================================================

  Logger::info("RouteFollower: INNER GATE");

  const double distanceToGate = calculateDistanceToGate(from, *gate);

  if(distanceToGate < 0.0 || distanceToGate > distance) {
    Logger::warning("RouteFollower: "
                    "invalid gate position");

    straight(distance);

    return;
  }

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "distanceToGate=%.2f mm",
                    distanceToGate);

  // =====================================================
  // QR①のマップ上推定位置
  //
  // QR①はゲート125mm手前。
  // =====================================================

  double nominalDistanceToQr1 = distanceToGate - QR_TO_GATE_DISTANCE;

  if(nominalDistanceToQr1 < 0.0) {
    nominalDistanceToQr1 = 0.0;
  }

  // =====================================================
  // 1回目：QR①
  // =====================================================

  Logger::info("RouteFollower: "
               "========== QR1 ==========");

  SquareAngleAdjustment::Result firstResult;

  const bool firstDetected = detectSquare(firstResult);

  if(firstDetected) {
    /*
     * QR①に向くための角度。
     */
    const double firstAngle = firstResult.correctionAngle;

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR1 angle=%.2f deg",
                      firstAngle);

    // QR①方向へ回頭
    rotate(firstAngle);

    /*
     * QR①真上まで移動。
     *
     * マップ上の固定値ではなく、
     * カメラ計測した実距離を使う。
     */
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "Move to QR1 %.2f mm",
                      firstResult.straightDistance);

    straight(firstResult.straightDistance);

    /*
     * QR①へ向くために回した分を戻す。
     *
     * +α
     * ↓
     * QR①へ移動
     * ↓
     * -α
     *
     * これで元のゲート通過方向へ戻す。
     */
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "Restore heading %.2f deg",
                      -firstAngle);

    rotate(-firstAngle);
  }

  // =====================================================
  // QR①検出失敗
  //
  // カメラ補正できなかった場合は、
  // マップ上のQR①位置まで通常走行。
  // =====================================================

  else {
    Logger::warning("RouteFollower: "
                    "QR1 detection failed "
                    "-> use map distance");

    straight(nominalDistanceToQr1);
  }

  // =====================================================
  // 2回目：QR②
  //
  // QR①真上へ移動し、
  // -α回頭で元方向へ戻した後に検出する。
  // =====================================================

  Logger::info("RouteFollower: "
               "========== QR2 ==========");

  SquareAngleAdjustment::Result secondResult;

  const bool secondDetected = detectSquare(secondResult);

  if(secondDetected) {
    /*
     * -αだけでは残る可能性がある
     * 実機誤差をQR②で補正する。
     */
    const double secondAngle = secondResult.correctionAngle;

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR2 correction angle=%.2f deg",
                      secondAngle);

    rotate(secondAngle);

    /*
     * QR②真上まで移動。
     *
     * 理論上はQR①→QR②=250mmだが、
     * 実際の位置・角度誤差を補正するため
     * カメラから求めた距離を使用する。
     */
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "Move to QR2 %.2f mm",
                      secondResult.straightDistance);

    straight(secondResult.straightDistance);
  }

  // =====================================================
  // QR②検出失敗
  //
  // QR①からQR②までは250mm。
  // =====================================================

  else {
    Logger::warning("RouteFollower: "
                    "QR2 detection failed "
                    "-> Straight 250mm");

    straight(QR_DISTANCE);
  }

  // =====================================================
  // QR②から区間終端まで
  //
  // マップ上でのQR②位置:
  //
  // ゲート中心 + 125mm
  // =====================================================

  const double nominalQr2Position = distanceToGate + QR_TO_GATE_DISTANCE;

  double remainingDistance = distance - nominalQr2Position;

  if(remainingDistance < 0.0) {
    remainingDistance = 0.0;
  }

  if(remainingDistance > 0.0) {
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "Remaining Straight %.2f mm",
                      remainingDistance);

    straight(remainingDistance);
  }

  Logger::info("RouteFollower: "
               "===== GATE SEGMENT FINISHED =====");
}

const Gate* RouteFollower::findGate(const RouteState& from, const RouteState& to) const
{
  for(const Gate& gate : mapData.getGates()) {
    const std::vector<GatePass> passes = mapData.getGatePasses(gate.color);

    for(const GatePass& pass : passes) {
      // =====================================================
      // Y方向の移動
      //
      // from
      //   |
      // entrance
      //   |
      // gate
      //   |
      // exit
      //   |
      // to
      // =====================================================

      if(from.x == to.x && pass.entrance.x == from.x && pass.exit.x == from.x) {
        // -------------------------------------------------
        // Yが増える方向
        // -------------------------------------------------

        if(to.y > from.y) {
          const bool entranceInside = pass.entrance.y >= from.y && pass.entrance.y <= to.y;

          const bool exitInside = pass.exit.y >= from.y && pass.exit.y <= to.y;

          const bool correctDirection = pass.exit.y > pass.entrance.y;

          if(entranceInside && exitInside && correctDirection) {
            Logger::printfLog(Logger::INFO,
                              "RouteFollower: "
                              "findGate MATCH "
                              "(%d,%d)->(%d,%d) "
                              "GatePass=(%d,%d)->(%d,%d)",
                              from.x, from.y, to.x, to.y, pass.entrance.x, pass.entrance.y,
                              pass.exit.x, pass.exit.y);

            return &gate;
          }
        }

        // -------------------------------------------------
        // Yが減る方向
        // -------------------------------------------------

        if(to.y < from.y) {
          const bool entranceInside = pass.entrance.y <= from.y && pass.entrance.y >= to.y;

          const bool exitInside = pass.exit.y <= from.y && pass.exit.y >= to.y;

          const bool correctDirection = pass.exit.y < pass.entrance.y;

          if(entranceInside && exitInside && correctDirection) {
            Logger::printfLog(Logger::INFO,
                              "RouteFollower: "
                              "findGate MATCH "
                              "(%d,%d)->(%d,%d) "
                              "GatePass=(%d,%d)->(%d,%d)",
                              from.x, from.y, to.x, to.y, pass.entrance.x, pass.entrance.y,
                              pass.exit.x, pass.exit.y);

            return &gate;
          }
        }
      }

      // =====================================================
      // X方向の移動
      // =====================================================

      if(from.y == to.y && pass.entrance.y == from.y && pass.exit.y == from.y) {
        // -------------------------------------------------
        // Xが増える方向
        // -------------------------------------------------

        if(to.x > from.x) {
          const bool entranceInside = pass.entrance.x >= from.x && pass.entrance.x <= to.x;

          const bool exitInside = pass.exit.x >= from.x && pass.exit.x <= to.x;

          const bool correctDirection = pass.exit.x > pass.entrance.x;

          if(entranceInside && exitInside && correctDirection) {
            Logger::printfLog(Logger::INFO,
                              "RouteFollower: "
                              "findGate MATCH "
                              "(%d,%d)->(%d,%d) "
                              "GatePass=(%d,%d)->(%d,%d)",
                              from.x, from.y, to.x, to.y, pass.entrance.x, pass.entrance.y,
                              pass.exit.x, pass.exit.y);

            return &gate;
          }
        }

        // -------------------------------------------------
        // Xが減る方向
        // -------------------------------------------------

        if(to.x < from.x) {
          const bool entranceInside = pass.entrance.x <= from.x && pass.entrance.x >= to.x;

          const bool exitInside = pass.exit.x <= from.x && pass.exit.x >= to.x;

          const bool correctDirection = pass.exit.x < pass.entrance.x;

          if(entranceInside && exitInside && correctDirection) {
            Logger::printfLog(Logger::INFO,
                              "RouteFollower: "
                              "findGate MATCH "
                              "(%d,%d)->(%d,%d) "
                              "GatePass=(%d,%d)->(%d,%d)",
                              from.x, from.y, to.x, to.y, pass.entrance.x, pass.entrance.y,
                              pass.exit.x, pass.exit.y);

            return &gate;
          }
        }
      }
    }
  }

  Logger::printfLog(Logger::WARNING,
                    "RouteFollower: "
                    "findGate NO MATCH "
                    "(%d,%d)->(%d,%d)",
                    from.x, from.y, to.x, to.y);

  return nullptr;
}
bool RouteFollower::isOuterGate(const Gate& gate) const
{
  /*
   * 横向きゲート
   */
  if(gate.start.y == gate.end.y) {
    const int gateY = gate.start.y;

    return gateY == 0 || gateY == SystemInfo::Y_GRID_NUM;
  }

  /*
   * 縦向きゲート
   */
  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;

    return gateX == 0 || gateX == SystemInfo::X_GRID_NUM;
  }

  return false;
}

double RouteFollower::calculateDistanceToGate(const RouteState& from, const Gate& gate) const
{
  const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);

  /*
   * 横向きゲート。
   *
   * ロボットはY方向に通過する。
   */
  if(gate.start.y == gate.end.y) {
    const int centerX = (gate.start.x + gate.end.x) / 2;

    const int gateY = gate.start.y;

    const EtRallyMap::Node gateNode = map.getNode(centerX, gateY);

    return std::abs(gateNode.y - fromNode.y);
  }

  /*
   * 縦向きゲート。
   *
   * ロボットはX方向に通過する。
   */
  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;

    const int centerY = (gate.start.y + gate.end.y) / 2;

    const EtRallyMap::Node gateNode = map.getNode(gateX, centerY);

    return std::abs(gateNode.x - fromNode.x);
  }

  Logger::error("RouteFollower: invalid gate");

  return -1.0;
}