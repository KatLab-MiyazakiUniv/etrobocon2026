/**
 * @file   ETZumoApproach.cpp
 * @brief  ボトルデリバリー後、QR補正して外周経由でET相撲開始位置へ移動する
 * @note   以前のETラリーにあったゲート情報不足時の固定最終経路を独立させたもの
 */
#include "ETZumoApproach.h"

#include <cmath>
#include <functional>
#include <memory>
#include <utility>

#include "AngleNormalizer.h"
#include "Course.h"
#include "CourseMirror.h"
#include "DistanceCondition.h"
#include "EtRallyMap.h"
#include "Logger.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "RouteTypes.h"
#include "SquareAngleAdjustment.h"
#include "Straight.h"
#include "SystemInfo.h"

namespace {
  // BaseMotion::run() から外部のCSV継続条件でスキップされず、一度だけ実行する。
  // EtRallyCompletionConditionとの名前の衝突を避けるため専用名にする。
  class ETZumoApproachOnceCondition : public BaseContinuationCondition {
   public:
    ETZumoApproachOnceCondition(Robot& robot, std::function<bool()> notCompleted)
      : BaseContinuationCondition(robot), notCompleted(std::move(notCompleted))
    {
    }

    bool shouldContinue() override { return notCompleted(); }

   private:
    std::function<bool()> notCompleted;
  };

  // 旧ETラリーの固定最終経路で使用していた値。
  constexpr double TARGET_SPEED = 300.0;
  constexpr double STRAIGHT_DEADBAND_RATE = 0.25;
  constexpr double STRAIGHT_MAXOUT_RATE = 0.54;
  constexpr double ROTATION_TOLERANCE = 2.0;
  constexpr double FINAL_STRAIGHT_DISTANCE = 100.0;

  Point convertPoint(const Point& point, Course course)
  {
    return course == Course::Right ? CourseMirror::mirrorPoint(point) : point;
  }

  Direction convertDirection(Direction direction, Course course)
  {
    return course == Course::Right ? CourseMirror::mirrorDirection(direction) : direction;
  }

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

  double calculateRotationAngle(Direction from, Direction to)
  {
    return AngleNormalizer::normalizeAngle(directionToHeading(from) - directionToHeading(to));
  }

  double calculateStraightDistance(const EtRallyMap& map, const Point& from, const Point& to)
  {
    const auto fromNode = map.getNode(from.x, from.y);
    const auto toNode = map.getNode(to.x, to.y);
    if(from.x == to.x) return std::abs(toNode.y - fromNode.y);
    if(from.y == to.y) return std::abs(toNode.x - fromNode.x);
    Logger::error("ETZumoApproach: diagonal route not supported");
    return 0.0;
  }

  void rotateRelative(Robot& robot, double angle, const Pid::PidGain& gain)
  {
    if(std::abs(angle) <= ROTATION_TOLERANCE) return;
    auto condition = std::make_unique<RelativeAngleCondition>(robot, angle, ROTATION_TOLERANCE);
    RelativeRotation motion(robot, std::move(condition), gain, angle);
    motion.run();
  }

  void moveStraight(Robot& robot, double distance, double speed, const Pid::PidGain& gain)
  {
    auto condition = std::make_unique<DistanceCondition>(robot, distance);
    Straight motion(robot, std::move(condition), speed, gain, true, STRAIGHT_DEADBAND_RATE,
                    STRAIGHT_MAXOUT_RATE);
    motion.run();
  }
}  // namespace

ETZumoApproach::ETZumoApproach(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _condition,
                               const Pid::PidGain& _rotationPid, const Pid::PidGain& _adjustmentPid,
                               const Pid::PidGain& _straightPid)
  : BaseMotion(_robot, std::make_unique<ETZumoApproachOnceCondition>(
                           _robot, [this]() { return !completed; })),
    rotationPid(_rotationPid),
    adjustmentPid(_adjustmentPid),
    straightPid(_straightPid)
{
  // 旧ETラリーと同じく独自の完了条件で1回だけ実行する。
  (void)_condition;
}

void ETZumoApproach::prepare()
{
  completed = false;
  successful = false;
}

void ETZumoApproach::executeStep()
{
  completed = true;
  Logger::info("ETZumoApproach: start");

  const Course course = robot.getCourse();
  EtRallyMap map;
  Point startPoint;
  Point outerPoint;

  // 以前のETラリーと同じラベル番号による開始位置判定（Lコース基準）。
  // ラベル番号とボトル色の対応はボトルデリバリー側の設定を使用する。
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

  // ========================================
  // QR補正（検出できなければ従来通り補正せず進む）
  // ========================================
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
      Logger::error("ETZumoApproach: invalid QR adjustment");
      return;
    }
    Logger::info("ETZumoApproach: square detected");
    rotateRelative(robot, result.correctionAngle, adjustmentPid);
    moveStraight(robot, result.straightDistance, TARGET_SPEED, straightPid);
    rotateRelative(robot, -result.correctionAngle, adjustmentPid);

    // QR補正後の位置から、車体の向きを変えずに外周へバックする。
    const double backDistance = calculateStraightDistance(map, startPoint, outerPoint);
    if(backDistance <= 0.0) {
      Logger::error("ETZumoApproach: invalid back distance");
      return;
    }
    moveStraight(robot, backDistance, -TARGET_SPEED, straightPid);
  } else {
    // 以前の処理と同じ仮定：QRを検出できなければ外周位置にいるものとして扱う。
    Logger::info("ETZumoApproach: square not detected -> start at outer point");
  }

  // ========================================
  // 外周位置 → (0,0) → (8,0) → 100mm直進
  // ========================================
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
  moveStraight(robot, distanceToCorner, TARGET_SPEED, straightPid);

  rotateRelative(robot, calculateRotationAngle(currentDirection, finalDirection), rotationPid);
  moveStraight(robot, distanceToFinal, TARGET_SPEED, straightPid);
  moveStraight(robot, FINAL_STRAIGHT_DISTANCE, TARGET_SPEED, straightPid);

  successful = true;
  Logger::info("ETZumoApproach: arrived at ETZumo start");
}

void ETZumoApproach::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}
