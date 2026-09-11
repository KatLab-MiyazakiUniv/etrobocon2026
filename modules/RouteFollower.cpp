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
   * @brief QR①とQR②の間隔[mm]
   */
  constexpr double QR_DISTANCE = 250.0;

  /**
   * @brief QRからゲート中心までの距離[mm]
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

  for(std::size_t i = 1; i < route.size(); ++i) {
    const RouteState& from = route[i - 1];

    const RouteState& to = route[i];

    Logger::printfLog(Logger::INFO,
                      "RouteFollower[%d]: "
                      "(%d,%d) -> (%d,%d)",
                      static_cast<int>(i), from.x, from.y, to.x, to.y);

    // =====================================================
    // 1. 通常の経路回頭
    // =====================================================

    const double rotationAngle = calculateRotationAngle(from.direction, to.direction);

    if(std::abs(rotationAngle) > ROTATION_TOLERANCE) {
      rotate(rotationAngle);
    }

    // =====================================================
    // 2. 同じ座標なら回頭だけなので終了
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
    // 4. ゲート区間か確認
    // =====================================================

    const Gate* gate = findGate(from, to);

    if(gate != nullptr) {
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

  return AngleNormalizer::normalizeAngle(currentHeading - targetHeading);
}

double RouteFollower::calculateDistance(const RouteState& from, const RouteState& to) const
{
  const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);

  const EtRallyMap::Node toNode = map.getNode(to.x, to.y);

  if(from.y == to.y) {
    return std::abs(toNode.x - fromNode.x);
  }

  if(from.x == to.x) {
    return std::abs(toNode.y - fromNode.y);
  }

  Logger::error("RouteFollower: diagonal route");

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
                    "RelativeRotation START %.2f deg",
                    angle);

  rotation.run();

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "RelativeRotation FINISHED %.2f deg",
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
  // カメラ撮影前に停止
  // =====================================================

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  // =====================================================
  // 検出範囲
  // =====================================================

  CameraServer::SquareDetectorRequest request{};

  request.roi.x = 0;

  request.roi.y = 0;

  request.roi.width = CAM_MAX_WIDTH;

  request.roi.height = CAM_MAX_HEIGHT;

  // =====================================================
  // 検出
  // =====================================================

  SquareAngleAdjustment adjustment(robot);

  result = adjustment.calculate(request);

  if(!result.wasDetected) {
    Logger::warning("RouteFollower: "
                    "square detection failed");

    return false;
  }

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "center=(%.2f, %.2f)",
                    result.centerX, result.centerY);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "forward=%.2f mm "
                    "lateral=%.2f mm",
                    result.forwardDistance, result.lateralDistance);

  Logger::printfLog(Logger::INFO,
                    "RouteFollower: "
                    "angle=%.2f deg "
                    "distance=%.2f mm",
                    result.correctionAngle, result.straightDistance);

  return true;
}

void RouteFollower::runGateSegment(const RouteState& from, const RouteState& to, double distance)
{
  Logger::info("RouteFollower: "
               "===== GATE SEGMENT START =====");

  // =====================================================
  // 1. ゲート取得
  // =====================================================

  const Gate* gate = findGate(from, to);

  if(gate == nullptr) {
    Logger::warning("RouteFollower: gate not found");

    straight(distance);

    return;
  }

  // =====================================================
  // 2. 外周ゲート
  //
  // 外周はQR補正なし。
  // =====================================================

  if(isOuterGate(*gate)) {
    Logger::info("RouteFollower: OUTER GATE");

    straight(distance);

    return;
  }

  Logger::info("RouteFollower: INNER GATE");

  // =====================================================
  // 3. ゲート中心までのマップ上距離
  // =====================================================

  const double distanceToGate = calculateDistanceToGate(from, *gate);

  if(distanceToGate < 0.0 || distanceToGate > distance) {
    Logger::warning("RouteFollower: "
                    "invalid gate position");

    straight(distance);

    return;
  }

  // =====================================================
  // 4. QR①のマップ上位置
  //
  // QR①
  //   ↓ 125mm
  // ゲート
  //   ↓ 125mm
  // QR②
  // =====================================================

  double nominalDistanceToQr1 = distanceToGate - QR_TO_GATE_DISTANCE;

  if(nominalDistanceToQr1 < 0.0) {
    nominalDistanceToQr1 = 0.0;
  }

  // =====================================================
  // 5. QR①検出
  // =====================================================

  Logger::info("RouteFollower: "
               "===== QR1 DETECTION =====");

  SquareAngleAdjustment::Result firstResult;

  const bool firstDetected = detectSquare(firstResult);

  // =====================================================
  // QR①検出成功
  // =====================================================

  double firstAngle = 0.0;

  if(firstDetected) {
    firstAngle = firstResult.correctionAngle;

    // ---------------------------------------------------
    // QR①へ向く
    // ---------------------------------------------------

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR1 rotate %.2f deg",
                      firstAngle);

    rotate(firstAngle);

    // ---------------------------------------------------
    // QR①の真上まで移動
    //
    // 画像から計算した実距離を使う。
    // ---------------------------------------------------

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "move to QR1 %.2f mm",
                      firstResult.straightDistance);

    straight(firstResult.straightDistance);

    // ---------------------------------------------------
    // QR①へ向くために回した角度を元へ戻す
    // ---------------------------------------------------

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "restore heading %.2f deg",
                      -firstAngle);

    rotate(-firstAngle);
  }

  // =====================================================
  // QR①検出失敗
  //
  // マップ上のQR①位置まで通常走行する。
  // =====================================================

  else {
    Logger::warning("RouteFollower: "
                    "QR1 detection failed "
                    "-> use map distance");

    straight(nominalDistanceToQr1);
  }

  // =====================================================
  // 6. QR②検出
  //
  // QR①の真上まで移動し、
  // -αで元の方向へ戻した後に検出する。
  // =====================================================

  Logger::info("RouteFollower: "
               "===== QR2 DETECTION =====");

  SquareAngleAdjustment::Result secondResult;

  const bool secondDetected = detectSquare(secondResult);

  // =====================================================
  // QR②検出成功
  // =====================================================

  if(secondDetected) {
    // ---------------------------------------------------
    // QR②に対して残っている角度誤差を補正
    // ---------------------------------------------------

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR2 correction %.2f deg",
                      secondResult.correctionAngle);

    rotate(secondResult.correctionAngle);

    // ---------------------------------------------------
    // QR②の真上まで移動
    //
    // QR①からQR②までは理論上250mmだが、
    // 実際の位置ずれを補正するため、
    // カメラから算出した距離を優先する。
    // ---------------------------------------------------

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "move to QR2 %.2f mm",
                      secondResult.straightDistance);

    straight(secondResult.straightDistance);
  }

  // =====================================================
  // QR②検出失敗
  //
  // QR①からQR②までは250mmなので、
  // 固定距離でゲートを通過する。
  // =====================================================

  else {
    Logger::warning("RouteFollower: "
                    "QR2 detection failed "
                    "-> Straight 250mm");

    straight(QR_DISTANCE);
  }

  // =====================================================
  // 7. QR②から区間終了地点までの残距離
  //
  // 区間開始
  //
  //   ↓
  //
  // QR①
  //
  //   125mm
  //
  // ゲート
  //
  //   125mm
  //
  // QR②
  //
  //   ↓
  //
  // 区間終了
  //
  // QR②のマップ上位置は
  //
  // distanceToGate + 125mm
  // =====================================================

  const double nominalQr2Position = distanceToGate + QR_TO_GATE_DISTANCE;

  double remainingDistance = distance - nominalQr2Position;

  if(remainingDistance < 0.0) {
    remainingDistance = 0.0;
  }

  // =====================================================
  // 8. 残距離
  // =====================================================

  if(remainingDistance > 0.0) {
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "remaining Straight %.2f mm",
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

    return gateY == 0 || gateY == SystemInfo::Y_GRID_NUM;
  }

  // =====================================================
  // 縦向きゲート
  // =====================================================

  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;

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
  // Y方向に通過
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
  // X方向に通過
  // =====================================================

  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;

    const int centerY = (gate.start.y + gate.end.y) / 2;

    const EtRallyMap::Node gateNode = map.getNode(gateX, centerY);

    return std::abs(gateNode.x - fromNode.x);
  }

  Logger::error("RouteFollower: invalid gate");

  return -1.0;
}