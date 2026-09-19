/**
 * @file   RouteFollower.cpp
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#include "RouteFollower.h"

namespace {

  constexpr double ROTATION_TOLERANCE = 0.5;                 // 回頭終了判定許容誤差[deg]
  constexpr double QR_DISTANCE = 250.0;                      // QR①とQR②の距離[mm]
  constexpr double QR_TO_GATE_DISTANCE = QR_DISTANCE / 2.0;  // QRとゲート中央の距離[mm]
  constexpr double SQUARE_DETECTION_DISTANCE = 250.0;        // QRを検出する基準距離[mm]
  constexpr int MOTION_SWITCH_WAIT = 200;                    // 動作切り替え待機時間[ms]

}  // namespace

RouteFollower::RouteFollower(Robot& _robot, const EtRallyMap& _map,
                             double _targetSpeed, const Pid::PidGain& _rotationPid,
                             const Pid::PidGain& _squareRotationPid, const Pid::PidGain& _rightPid,
                             const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid)
  : robot(_robot),
    map(_map),
    targetSpeed(_targetSpeed),
    rotationPid(_rotationPid),
    squareRotationPid(_squareRotationPid),
    rightPid(_rightPid),
    leftPid(_leftPid),
    straightAnglePid(_straightAnglePid)
{
  LOG_CREATE("RouteFollower");
}

void RouteFollower::run(const std::vector<RouteState>& route)
{
  if(route.size() < 2) {
    Logger::warning("RouteFollower: 経路が2点未満のため走行できません");
    return;
  }

  for(std::size_t i = 1; i < route.size(); ++i) {
    // 区間の開始地点と終了地点を取得する
    const RouteState& from = route[i - 1];
    const RouteState& to = route[i];

    Logger::printfLog(
  Logger::INFO,
  "RouteFollower[%d]: 現在走行する区間 (%d,%d) → (%d,%d)",
  static_cast<int>(i),
  from.x, from.y,
  to.x, to.y);

    // 次の区間を走行するために必要な回頭角度を計算する
    const double rotationAngle = calculateRotationAngle(from.direction, to.direction);

    // 区間開始時に回頭が必要だったかを判定する
    const bool rotatedAtSegmentStart = std::abs(rotationAngle) > ROTATION_TOLERANCE;

    if(rotatedAtSegmentStart) {
      Logger::printfLog(Logger::INFO, "RouteFollower: 回頭開始 %.2f deg", rotationAngle);

      // 次の区間の進行方向へ回頭する
      rotate(rotationAngle);
    }

    // 同一座標の場合は、回頭のみ行って次の区間へ進む。
    if(from.x == to.x && from.y == to.y) {
      continue;
    }

    // 区間の走行距離を計算する。
    const double distance = calculateDistance(from, to);

    if(distance <= 0.0) {
      Logger::error("RouteFollower: 区間距離が不正です");

      // 異常な距離の場合はモーターを停止する。
      robot.getWheelMotorControllerInstance().stopBoth();

      return;
    }
Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "区間距離=%.2f mm",
  distance);

//ゲート検索
const Gate* gate = findGate(from, to);

if(gate != nullptr) {
  Logger::info("RouteFollower: "
               "ゲートを検出しました");

  runGateSegment(from, to, distance, rotatedAtSegmentStart);

      continue;
    }
    Logger::info("RouteFollower: 通常区間");

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

  // X方向
  if(from.y == to.y) {
    return std::abs(toNode.x - fromNode.x);
  }

  // Y方向
  if(from.x == to.x) {
    return std::abs(toNode.y - fromNode.y);
  }

  Logger::printfLog(
  Logger::ERROR,
  "RouteFollower: "
  "斜めの経路 "
  "(%d,%d) -> (%d,%d)",
  from.x, from.y, to.x, to.y);

  return 0.0;
}

void RouteFollower::rotate(double angle)
{
  if(std::abs(angle) <= ROTATION_TOLERANCE) {
    return;
  }

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  auto condition = std::make_unique<RelativeAngleCondition>(robot, angle, ROTATION_TOLERANCE);

  RelativeRotation rotation(robot, std::move(condition), rotationPid, angle);

  Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "回頭開始 %.2f 度",
  angle);

rotation.run();

Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "回頭完了 %.2f 度",
  angle);
}

void RouteFollower::rotateForSquare(double angle)
{
  if(std::abs(angle) <= ROTATION_TOLERANCE) {
   Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "正方形回頭をスキップ %.2f 度",
  angle);

    return;
  }

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  auto condition = std::make_unique<RelativeAngleCondition>(robot, angle, ROTATION_TOLERANCE);

  RelativeRotation rotation(robot, std::move(condition), squareRotationPid, angle);
Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "正方形回頭開始 %.2f 度",
  angle);

rotation.run();

Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "正方形回頭完了 %.2f 度",
  angle);
}

void RouteFollower::straight(double distance)
{
  if(distance <= 0.0) {
    return;
  }

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  auto condition = std::make_unique<DistanceCondition>(robot, distance);

  Straight straightMotion(robot, std::move(condition), targetSpeed, rightPid, leftPid,
                          straightAnglePid, true);
Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "直進開始 %.2f mm",
  distance);

straightMotion.run();

Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "直進完了 %.2f mm",
  distance);
}

bool RouteFollower::detectSquare(SquareAngleAdjustment::Result& result)
{
  Logger::info("RouteFollower: "
             "detectSquare 呼び出し");

  robot.getWheelMotorControllerInstance().stopBoth();

  ClockUtil::sleep(MOTION_SWITCH_WAIT);

  // 正方形検出要求
  CameraServer::SquareDetectorRequest request{};

  request.roi.x = 0;
  request.roi.y = 0;
  request.roi.width = CAM_MAX_WIDTH;
  request.roi.height = CAM_MAX_HEIGHT;

  SquareAngleAdjustment adjustment(robot);

  result = adjustment.calculate(request);

  if(!result.wasDetected) {
    Logger::warning("RouteFollower: "
                    "Square detection FAILED");

    return false;
  }
Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "正方形中心=(%.2f, %.2f)",
  result.centerX, result.centerY);

Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "前方=%.2f "
  "横方向=%.2f",
  result.forwardDistance, result.lateralDistance);

Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "角度=%.2f "
  "距離=%.2f",
  result.correctionAngle, result.straightDistance);
  return true;
}

void RouteFollower::runGateSegment(const RouteState& from, const RouteState& to, double distance,
                                   bool rotatedAtSegmentStart)
{
  Logger::info("RouteFollower: "
               "runGateSegment CALLED");

  const Gate* gate = findGate(from, to);

  if(gate == nullptr) {
    straight(distance);

    return;
  }

  if(isOuterGate(*gate)) {
Logger::info("RouteFollower: 外側ゲート");
    straight(distance);
    return;
  }

  // 内側ゲート
Logger::info("RouteFollower: 内側ゲート");
  // ゲート中心までの距離
  const double distanceToGate = calculateDistanceToGate(from, *gate);

  Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "ゲートまでの距離=%.2f mm",
  distanceToGate);

if(distanceToGate < 0.0 || distanceToGate > distance) {
  Logger::warning("RouteFollower: "
                  "ゲート位置が不正です");

    straight(distance);

    return;
  }

  double distanceToQr1 = distanceToGate - QR_TO_GATE_DISTANCE;

  if(distanceToQr1 < 0.0) {
    distanceToQr1 = 0.0;
  }

  const double distanceToFirstDetection = distanceToQr1 - SQUARE_DETECTION_DISTANCE;

  // QR①補正スキップ判定
  const bool skipFirstCorrection = rotatedAtSegmentStart && distanceToFirstDetection <= 0.0;

Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "区間開始時に回頭=%d "
  "最初の検出までの距離=%.2f "
  "最初の補正をスキップ=%d",
  rotatedAtSegmentStart ? 1 : 0,
  distanceToFirstDetection,
  skipFirstCorrection ? 1 : 0);
  
  // QR①補正
  if(!skipFirstCorrection) {
    double firstMoveDistance = distanceToFirstDetection;

    if(firstMoveDistance < 0.0) {
      firstMoveDistance = 0.0;
    }

   Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "QR①検出位置まで移動 "
  "%.2f mm",
  firstMoveDistance);

straight(firstMoveDistance);

// QR①検出
SquareAngleAdjustment::Result firstResult{};

    const bool firstDetected = detectSquare(firstResult);

    // QR①検出成功
    if(firstDetected) {
      const double firstAngle = firstResult.correctionAngle;

     Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "QR①補正角度=%.2f 度",
  firstAngle);

// QR①方向へ回頭
rotateForSquare(firstAngle);

// QR①まで進む
Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "QR①まで移動 %.2f mm",
  firstResult.straightDistance);
      straight(firstResult.straightDistance);

Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "QR①補正角度を戻す %.2f 度",
  -firstAngle);

      rotateForSquare(-firstAngle);
    }

    // QR①検出失敗
    else {
     Logger::warning("RouteFollower: "
                "QR①の検出に失敗しました "
                "→ 250mm直進");

      straight(SQUARE_DETECTION_DISTANCE);
    }

   Logger::info("RouteFollower: "
             "QR①位置 = QR②検出位置 "
             "→ 追加直進をスキップ");
  }

  // QR①をスキップする場合
  else {
 Logger::warning("RouteFollower: "
                "QR①補正をスキップ");

    double distanceToSecondDetection
        = distanceToGate + QR_TO_GATE_DISTANCE - SQUARE_DETECTION_DISTANCE;

    if(distanceToSecondDetection < 0.0) {
      distanceToSecondDetection = 0.0;
    }

    Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "QR②検出位置まで直接移動 %.2f mm",
  distanceToSecondDetection);

    if(distanceToSecondDetection > 0.0) {
      straight(distanceToSecondDetection);
    }
  }

 Logger::info("RouteFollower: "
             "QR②");
  SquareAngleAdjustment::Result secondResult{};

  const bool secondDetected = detectSquare(secondResult);

  // QR②検出成功
  if(secondDetected) {
    const double secondAngle = secondResult.correctionAngle;

    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "QR2 correction angle=%.2f deg",
                      secondAngle);

    // QR②方向へ回頭
    rotateForSquare(secondAngle);

    // QR②まで進む
    Logger::printfLog(Logger::INFO,
                      "RouteFollower: "
                      "Move to QR2 %.2f mm",
                      secondResult.straightDistance);

    straight(secondResult.straightDistance);
  }

  // QR②検出失敗


  else {
Logger::warning("RouteFollower: "
                "QR②の検出に失敗しました "
                "→ 回頭なし");

Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "QR②検出失敗時の直進 %.2f mm",
  SQUARE_DETECTION_DISTANCE);
    straight(SQUARE_DETECTION_DISTANCE);
  }

  // QR②から区間終端まで
  const double nominalQr2Position = distanceToGate + QR_TO_GATE_DISTANCE;

  double remainingDistance = distance - nominalQr2Position;

  if(remainingDistance < 0.0) {
    remainingDistance = 0.0;
  }

 Logger::printfLog(
  Logger::INFO,
  "RouteFollower: "
  "残り距離=%.2f mm",
  remainingDistance);

if(remainingDistance > 0.0) {
  straight(remainingDistance);
}

Logger::info("RouteFollower: "
             "ゲート区間走行完了");
}

const Gate* RouteFollower::findGate(const RouteState& from, const RouteState& to) const
{
  for(const Gate& gate : robot.getMapData().getGates()) {
    const std::vector<GatePass> passes = robot.getMapData().getGatePasses(gate.color);
    for(const GatePass& pass : passes) {
      // Y方向
      if(from.x == to.x && pass.entrance.x == from.x && pass.exit.x == from.x) {
        // Y増加
        if(to.y > from.y) {
          const bool entranceInside = pass.entrance.y >= from.y && pass.entrance.y <= to.y;
          const bool exitInside = pass.exit.y >= from.y && pass.exit.y <= to.y;
          const bool correctDirection = pass.exit.y > pass.entrance.y;

          if(entranceInside && exitInside && correctDirection) {
            return &gate;
          }
        }

        // Y減少
        if(to.y < from.y) {
          const bool entranceInside = pass.entrance.y <= from.y && pass.entrance.y >= to.y;
          const bool exitInside = pass.exit.y <= from.y && pass.exit.y >= to.y;
          const bool correctDirection = pass.exit.y < pass.entrance.y;

          if(entranceInside && exitInside && correctDirection) {
            return &gate;
          }
        }
      }

      // X方向
      if(from.y == to.y && pass.entrance.y == from.y && pass.exit.y == from.y) {
        // X増加
        if(to.x > from.x) {
          const bool entranceInside = pass.entrance.x >= from.x && pass.entrance.x <= to.x;
          const bool exitInside = pass.exit.x >= from.x && pass.exit.x <= to.x;
          const bool correctDirection = pass.exit.x > pass.entrance.x;

          if(entranceInside && exitInside && correctDirection) {
            return &gate;
          }
        }

        // X減少
        if(to.x < from.x) {
          const bool entranceInside = pass.entrance.x <= from.x && pass.entrance.x >= to.x;
          const bool exitInside = pass.exit.x <= from.x && pass.exit.x >= to.x;
          const bool correctDirection = pass.exit.x < pass.entrance.x;

          if(entranceInside && exitInside && correctDirection) {
            return &gate;
          }
        }
      }
    }
  }

  return nullptr;
}

bool RouteFollower::isOuterGate(const Gate& gate) const
{
  // 横向きゲート
  if(gate.start.y == gate.end.y) {
    const int gateY = gate.start.y;

    return gateY == 0 || gateY == SystemInfo::Y_GRID_NUM;
  }

  // 縦向きゲート
  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;

    return gateX == 0 || gateX == SystemInfo::X_GRID_NUM;
  }

  return false;
}

double RouteFollower::calculateDistanceToGate(const RouteState& from, const Gate& gate) const
{
  const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);

  // 横向きゲート
  if(gate.start.y == gate.end.y) {
    const int centerX = (gate.start.x + gate.end.x) / 2;
    const int gateY = gate.start.y;
    const EtRallyMap::Node gateNode = map.getNode(centerX, gateY);

    return std::abs(gateNode.y - fromNode.y);
  }

  // 縦向きゲート
  if(gate.start.x == gate.end.x) {
    const int gateX = gate.start.x;
    const int centerY = (gate.start.y + gate.end.y) / 2;
    const EtRallyMap::Node gateNode = map.getNode(gateX, centerY);

    return std::abs(gateNode.x - fromNode.x);
  }

Logger::error("RouteFollower: 不正なゲート");

  return -1.0;
}