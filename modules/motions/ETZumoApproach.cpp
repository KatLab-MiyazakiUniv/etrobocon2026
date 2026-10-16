/**
 * @file   ETZumoApproach.cpp
 * @brief  ボトルデリバリー後、QR補正して外周経由でET相撲開始位置へ移動する
 * @author okuyama0528
 */

#include "ETZumoApproach.h"

namespace {
  constexpr double ROTATION_TOLERANCE
      = 2.0;  // 回頭時の許容誤差(補正角度がこの値以下なら回頭しない)
  constexpr double FINAL_STRAIGHT_DISTANCE
      = 100.0;  // 外周経由でET相撲開始位置までの最終直進距離(mm)

  // 指定された座標をコースに応じて変換する
  Point convertPoint(const Point& point, Course course)
  {
    return course == Course::Right ? CourseMirror::mirrorPoint(point) : point;
  }

  // 指定された方向をコースに応じて変換する
  Direction convertDirection(Direction direction, Course course)
  {
    return course == Course::Right ? CourseMirror::mirrorDirection(direction) : direction;
  }

  // 指定された方向を角度に変換する
  double directionToHeading(Direction direction)
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

  // 指定された2つの方向の差分角度を計算する
  double calculateRotationAngle(Direction from, Direction to)
  {
    return AngleNormalizer::normalizeAngle(directionToHeading(from) - directionToHeading(to));
  }

  // 指定された2つの座標間の直線距離を計算する
  double calculateStraightDistance(const EtRallyMap& map, const Point& from, const Point& to)
  {
    const auto fromNode = map.getNode(from.x, from.y);
    const auto toNode = map.getNode(to.x, to.y);
    if(from.x == to.x) return std::abs(toNode.y - fromNode.y);
    if(from.y == to.y) return std::abs(toNode.x - fromNode.x);
    Logger::error("ETZumoApproach: diagonal route not supported");
    return 0.0;
  }

  // 指定された角度だけ相対回転する
  void rotateRelative(Robot& robot, double angle, const Pid::PidGain& gain)
  {
    if(std::abs(angle) <= ROTATION_TOLERANCE) return;
    auto condition = std::make_unique<RelativeAngleCondition>(robot, angle, ROTATION_TOLERANCE);
    RelativeRotation motion(robot, std::move(condition), gain, angle);
    motion.run();
  }

  // 指定された距離だけ直進する
  void moveStraight(Robot& robot, double distance, double speed, const Pid::PidGain& gain,
                    double deadbandRate, double maxoutRate)
  {
    auto condition = std::make_unique<DistanceCondition>(robot, distance);
    Straight motion(robot, std::move(condition), speed, gain, true, deadbandRate, maxoutRate);
    motion.run();
  }
}  // namespace

ETZumoApproach::ETZumoApproach(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _condition,
                               const Pid::PidGain& _rotationPid, const Pid::PidGain& _adjustmentPid,
                               const Pid::PidGain& _straightPid, double _targetSpeed,
                               double _straightDeadbandRate, double _straightMaxoutRate)
  : BaseMotion(_robot, std::move(_condition)),
    rotationPid(_rotationPid),
    adjustmentPid(_adjustmentPid),
    straightPid(_straightPid),
    targetSpeed(_targetSpeed),
    straightDeadbandRate(_straightDeadbandRate),
    straightMaxoutRate(_straightMaxoutRate)
{
  LOG_CREATE("ETZumoApproach");
}

void ETZumoApproach::prepare() {}

void ETZumoApproach::executeStep()
{
  const Course course = robot.getCourse();
  EtRallyMap map;
  Point startPoint;
  Point outerPoint;

  // ラベル番号に応じてET相撲開始位置を設定する
  switch(robot.getIndexOfLabel()) {
    case 0:
      startPoint = convertPoint({ 2, 4 }, course);
      outerPoint = convertPoint({ 0, 4 }, course);
      break;
    case 1:
      startPoint = convertPoint({ 2, 6 }, course);
      outerPoint = convertPoint({ 0, 6 }, course);
      break;
    case 2:
      startPoint = convertPoint({ 2, 4 }, course);
      outerPoint = convertPoint({ 0, 4 }, course);
      break;
    default:
      Logger::error("ETZumoApproach: invalid label index");
      return;
  }

  // QR補正（検出できなければ従来通り補正せず進む）
  CameraServer::SquareDetectorRequest request{};
  request.roi.x = 0;
  request.roi.y = 0;
  request.roi.width = CAM_MAX_WIDTH;
  request.roi.height = CAM_MAX_HEIGHT;
  SquareAngleAdjustment adjustment(robot);
  const auto result = adjustment.calculate(request);

  if(result.wasDetected) {
    if(!std::isfinite(result.correctionAngle) || !std::isfinite(result.straightDistance)
       || result.straightDistance <= 0.0) {
      Logger::error("ETZumoApproach: 正方形が検出されましたが、補正角度または直進距離が無効です");
      return;
    }
    Logger::info("ETZumoApproach:正方形が検出されました");
    rotateRelative(robot, result.correctionAngle, adjustmentPid);
    moveStraight(robot, result.straightDistance, targetSpeed, straightPid, straightDeadbandRate,
                 straightMaxoutRate);
    rotateRelative(robot, -result.correctionAngle, adjustmentPid);

    // QR補正後の位置から、車体の向きを変えずに外周へバックする。
    const double backDistance = calculateStraightDistance(map, startPoint, outerPoint);
    if(backDistance <= 0.0) {
      Logger::error("ETZumoApproach:後退距離が無効です");
      return;
    }
    moveStraight(robot, backDistance, -targetSpeed, straightPid, straightDeadbandRate,
                 straightMaxoutRate);
  } else {
    // 以前の処理と同じ仮定：QRを検出できなければ外周位置にいるものとして扱う。
    Logger::info(
        "ETZumoApproach:正方形が検出されませんでした。外周位置にいるものとして処理を続行します");
  }

  // 外周位置 → (0,0) → (8,0) → 100mm直進
  const Point cornerPoint = convertPoint({ 0, 0 }, course);
  const Point finalPoint = convertPoint({ 8, 0 }, course);
  Direction currentDirection = convertDirection(Direction::LEFT, course);
  const Direction cornerDirection = convertDirection(Direction::UP, course);
  const Direction finalDirection = convertDirection(Direction::LEFT, course);

  const double distanceToCorner = calculateStraightDistance(map, outerPoint, cornerPoint);
  const double distanceToFinal = calculateStraightDistance(map, cornerPoint, finalPoint);
  if(distanceToCorner <= 0.0 || distanceToFinal <= 0.0) {
    Logger::error("ETZumoApproach: invalid outer route distance");
    return;
  }

  rotateRelative(robot, calculateRotationAngle(currentDirection, cornerDirection), rotationPid);
  currentDirection = cornerDirection;
  moveStraight(robot, distanceToCorner, targetSpeed, straightPid, straightDeadbandRate,
               straightMaxoutRate);
  rotateRelative(robot, calculateRotationAngle(currentDirection, finalDirection), rotationPid);
  moveStraight(robot, distanceToFinal, targetSpeed, straightPid, straightDeadbandRate,
               straightMaxoutRate);
  moveStraight(robot, FINAL_STRAIGHT_DISTANCE, targetSpeed, straightPid, straightDeadbandRate,
               straightMaxoutRate);
  Logger::info("ETZumoApproach: arrived at ETZumo start");
}

void ETZumoApproach::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}
