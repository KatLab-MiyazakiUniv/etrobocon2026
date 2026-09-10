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
  constexpr double SQUARE_CENTER_TOLERANCE = 10.0;

  /**
   * @brief 1回目の正方形補正を行うゲート手前距離[mm]
   */
  constexpr double FIRST_SQUARE_ADJUSTMENT_BEFORE_GATE = 375.0;

  /**
   * @brief 2回目の正方形補正を行うゲート手前距離[mm]
   */
  constexpr double SECOND_SQUARE_ADJUSTMENT_BEFORE_GATE = 125.0;

  /**
   * @brief 回頭を実行する最低角度[deg]
   */
  constexpr double MIN_ROTATION_ANGLE = 0.001;

  /**
   * @brief 制御変更前の停止時間[ms]
   */
  constexpr int MOTION_CHANGE_SLEEP_TIME = 200;

  /**
   * @brief 制御変更前に停止して待機する
   */
  void waitBeforeMotion(Robot& robot)
  {
    robot.getWheelMotorControllerInstance().stopBoth();

    ClockUtil::sleep(MOTION_CHANGE_SLEEP_TIME);
  }

}  // namespace

RouteFollower::RouteFollower(Robot& _robot, const EtRallyMap& _map, const MapData& _mapData,
                             double _targetSpeed, const Pid::PidGain& _rotationPid,
                             const Pid::PidGain& _rightPid, const Pid::PidGain& _leftPid,
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
    squareAnglePid(_squareAnglePid)
{
  LOG_CREATE("RouteFollower");
}

void RouteFollower::run(const std::vector<RouteState>& route)
{
  if(route.size() < 2) {
    Logger::info("RouteFollower: route is too short");

    return;
  }

  for(std::size_t i = 1; i < route.size(); ++i) {
    const RouteState& from = route[i - 1];
    const RouteState& to = route[i];

    Logger::printfLog(Logger::INFO, "RouteFollower[%d]: (%d,%d) -> (%d,%d)", static_cast<int>(i),
                      from.x, from.y, to.x, to.y);

    // =====================================================
    // 1. 回頭
    // =====================================================

    const double rotationAngle = calculateRotationAngle(from.direction, to.direction);

    if(std::abs(rotationAngle) > MIN_ROTATION_ANGLE) {
      Logger::printfLog(Logger::INFO, "RouteFollower: rotation required %.2f deg", rotationAngle);

      /*
       * 通常の回頭では正方形補正を行わない。
       *
       * 正方形補正はゲート区間内でのみ行う。
       */
      rotate(rotationAngle);
    }

    // =====================================================
    // 2. 方向変更のみ
    // =====================================================

    if(from.x == to.x && from.y == to.y) {
      continue;
    }

    // =====================================================
    // 3. 区間距離
    // =====================================================

    const double distance = calculateDistance(from, to);

    if(distance <= 0.0) {
      Logger::error("RouteFollower: invalid distance");

      robot.getWheelMotorControllerInstance().stopBoth();

      return;
    }

    // =====================================================
    // 4. ゲート区間判定
    // =====================================================

    const Gate* gate = findGate(from, to);

    /*
     * ゲートを通らない通常区間。
     */
    if(gate == nullptr) {
      straight(distance);

      continue;
    }

    // =====================================================
    // 5. ゲート区間
    // =====================================================

    Logger::info("RouteFollower: ===== GATE SEGMENT =====");

    runGateSegment(from, to, distance);
  }

  robot.getWheelMotorControllerInstance().stopBoth();
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

  return AngleNormalizer::normalizeAngle(currentHeading - targetHeading);
}

double RouteFollower::calculateDistance(const RouteState& from, const RouteState& to) const
{
  const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);

  const EtRallyMap::Node toNode = map.getNode(to.x, to.y);

  /*
   * X方向移動
   */
  if(from.y == to.y) {
    return std::abs(toNode.x - fromNode.x);
  }

  /*
   * Y方向移動
   */
  if(from.x == to.x) {
    return std::abs(toNode.y - fromNode.y);
  }

  Logger::printfLog(Logger::ERROR, "RouteFollower: diagonal route (%d,%d) -> (%d,%d)", from.x,
                    from.y, to.x, to.y);

  return 0.0;
}

void RouteFollower::rotate(double angle)
{
  if(std::abs(angle) <= MIN_ROTATION_ANGLE) {
    return;
  }

  // =====================================================
  // 制御変更前停止
  // =====================================================

  waitBeforeMotion(robot);

  auto condition = std::make_unique<RelativeAngleCondition>(robot, angle, ROTATION_TOLERANCE);

  RelativeRotation rotation(robot, std::move(condition), rotationPid, angle);

  Logger::printfLog(Logger::INFO, "RouteFollower: Rotation START %.2f", angle);

  rotation.run();

  Logger::printfLog(Logger::INFO, "RouteFollower: Rotation FINISHED %.2f", angle);
}

void RouteFollower::straight(double distance)
{
  if(distance <= 0.0) {
    return;
  }

  // =====================================================
  // 制御変更前停止
  // =====================================================

  waitBeforeMotion(robot);

  auto condition = std::make_unique<DistanceCondition>(robot, distance);

  Straight straightMotion(robot, std::move(condition), targetSpeed, rightPid, leftPid,
                          straightAnglePid, true);

  Logger::printfLog(Logger::INFO, "RouteFollower: Straight START %.2f", distance);

  straightMotion.run();

  Logger::printfLog(Logger::INFO, "RouteFollower: Straight FINISHED %.2f", distance);
}

bool RouteFollower::adjustAngleWithSquare()
{
  // =====================================================
  // 制御変更前停止
  // =====================================================

  Logger::info("RouteFollower: wait before SquareAngleAdjustment");

  waitBeforeMotion(robot);

  // =====================================================
  // ROI
  // =====================================================

  CameraServer::SquareDetectorRequest squareRequest{};

  /*
   * 角度補正では正方形が左右へ大きくずれている可能性があるため、
   * 画像全体を検出範囲とする。
   */
  squareRequest.roi.x = 0;
  squareRequest.roi.y = 0;
  squareRequest.roi.width = CAM_MAX_WIDTH;
  squareRequest.roi.height = CAM_MAX_HEIGHT;

  // =====================================================
  // SquareAngleAdjustment
  // =====================================================

  SquareAngleAdjustment adjustment(robot, squareRequest, squareAnglePid, SQUARE_CENTER_TOLERANCE);

  Logger::info("RouteFollower: "
               "========== SquareAngleAdjustment START ==========");

  const bool success = adjustment.run();

  if(!success) {
    Logger::warning("RouteFollower: "
                    "========== SquareAngleAdjustment FAILED ==========");

    return false;
  }

  Logger::info("RouteFollower: "
               "========== SquareAngleAdjustment SUCCESS ==========");

  return true;
}

const Gate* RouteFollower::findGate(const RouteState& from, const RouteState& to) const
{
  /*
   * MapDataに登録されているすべてのゲートを確認する。
   */
  for(const Gate& gate : mapData.getGates()) {
    const std::vector<GatePass> passes = mapData.getGatePasses(gate.color);

    /*
     * 各ゲートには両方向のGatePassが存在する。
     */
    for(const GatePass& pass : passes) {
      const bool entranceMatches = pass.entrance.x == from.x && pass.entrance.y == from.y;

      const bool exitMatches = pass.exit.x == to.x && pass.exit.y == to.y;

      if(entranceMatches && exitMatches) {
        return &gate;
      }
    }
  }

  return nullptr;
}

bool RouteFollower::isOuterGate(const Gate& gate) const
{
  // =====================================================
  // 横向きゲート
  // =====================================================

  if(gate.start.y == gate.end.y) {
    const int gateY = gate.start.y;

    /*
     * 上端または下端にあるゲートは外周。
     */
    return gateY == 0 || gateY == SystemInfo::Y_GRID_NUM;
  }

  // =====================================================
  // 縦向きゲート
  // =====================================================

  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;

    /*
     * 左端または右端にあるゲートは外周。
     */
    return gateX == 0 || gateX == SystemInfo::X_GRID_NUM;
  }

  return false;
}

double RouteFollower::calculateDistanceToGate(const RouteState& from, const Gate& gate) const
{
  const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);

  // =====================================================
  // 横向きゲート
  //
  // Y方向へ通過する。
  // =====================================================

  if(gate.start.y == gate.end.y) {
    const int centerX = (gate.start.x + gate.end.x) / 2;

    const int gateY = gate.start.y;

    const EtRallyMap::Node gateNode = map.getNode(centerX, gateY);

    return std::abs(gateNode.y - fromNode.y);
  }

  // =====================================================
  // 縦向きゲート
  //
  // X方向へ通過する。
  // =====================================================

  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;

    const int centerY = (gate.start.y + gate.end.y) / 2;

    const EtRallyMap::Node gateNode = map.getNode(gateX, centerY);

    return std::abs(gateNode.x - fromNode.x);
  }

  Logger::error("RouteFollower: invalid gate");

  return 0.0;
}

void RouteFollower::runGateSegment(const RouteState& from, const RouteState& to, double distance)
{
  // =====================================================
  // 1. 通過するゲートを取得
  // =====================================================

  const Gate* gate = findGate(from, to);

  if(gate == nullptr) {
    Logger::warning("RouteFollower: gate not found -> Straight");

    straight(distance);

    return;
  }

  // =====================================================
  // 2. 外周ゲート
  // =====================================================

  /*
   * 外周ゲートでは正方形補正を使用しない。
   */
  if(isOuterGate(*gate)) {
    Logger::info("RouteFollower: outer gate -> Straight");

    straight(distance);

    return;
  }

  // =====================================================
  // 3. ゲートまでの距離
  // =====================================================

  const double distanceToGate = calculateDistanceToGate(from, *gate);

  Logger::printfLog(Logger::INFO, "RouteFollower: distance to gate = %.2f mm", distanceToGate);

  if(distanceToGate <= 0.0) {
    Logger::warning("RouteFollower: invalid distance to gate -> Straight");

    straight(distance);

    return;
  }

  /*
   * この関数内で既に走行した距離。
   */
  double traveledDistance = 0.0;

  // =====================================================
  // 4. 1回目の補正地点までStraight
  //
  // ゲート375mm手前。
  // =====================================================

  const double firstAdjustmentPosition = distanceToGate - FIRST_SQUARE_ADJUSTMENT_BEFORE_GATE;

  if(firstAdjustmentPosition > traveledDistance) {
    const double firstStraightDistance = firstAdjustmentPosition - traveledDistance;

    Logger::printfLog(Logger::INFO, "RouteFollower: Straight to FIRST square adjustment %.2f mm",
                      firstStraightDistance);

    straight(firstStraightDistance);

    traveledDistance += firstStraightDistance;
  }

  // =====================================================
  // 5. 1回目 SquareAngleAdjustment
  // =====================================================

  /*
   * 開始地点が既に375mm以内の場合でも、
   * ゲート125mm手前より外側であれば現在位置で補正する。
   */
  if(distanceToGate > SECOND_SQUARE_ADJUSTMENT_BEFORE_GATE) {
    Logger::info("RouteFollower: "
                 "===== FIRST SquareAngleAdjustment =====");

    const bool firstAdjusted = adjustAngleWithSquare();

    if(!firstAdjusted) {
      /*
       * 正方形が見つからなくても走行自体は続ける。
       */
      Logger::warning("RouteFollower: "
                      "FIRST SquareAngleAdjustment failed -> continue");
    }
  }

  // =====================================================
  // 6. 2回目の補正地点までStraight
  //
  // ゲート125mm手前。
  // =====================================================

  const double secondAdjustmentPosition = distanceToGate - SECOND_SQUARE_ADJUSTMENT_BEFORE_GATE;

  if(secondAdjustmentPosition > traveledDistance) {
    const double secondStraightDistance = secondAdjustmentPosition - traveledDistance;

    Logger::printfLog(Logger::INFO, "RouteFollower: Straight to SECOND square adjustment %.2f mm",
                      secondStraightDistance);

    straight(secondStraightDistance);

    traveledDistance += secondStraightDistance;
  }

  // =====================================================
  // 7. 2回目 SquareAngleAdjustment
  // =====================================================

  if(distanceToGate >= SECOND_SQUARE_ADJUSTMENT_BEFORE_GATE) {
    Logger::info("RouteFollower: "
                 "===== SECOND SquareAngleAdjustment =====");

    const bool secondAdjusted = adjustAngleWithSquare();

    if(!secondAdjusted) {
      /*
       * 2回目の補正に失敗しても停止せず、
       * Straightでゲートを通過する。
       */
      Logger::warning("RouteFollower: "
                      "SECOND SquareAngleAdjustment failed -> continue");
    }
  }

  // =====================================================
  // 8. 区間終了までStraight
  // =====================================================

  const double remainingDistance = distance - traveledDistance;

  if(remainingDistance > 0.0) {
    Logger::printfLog(Logger::INFO, "RouteFollower: final Straight through gate %.2f mm",
                      remainingDistance);

    straight(remainingDistance);
  }
}