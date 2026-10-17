/**
 * @file   EtRallyMotion.cpp
 * @brief  ETラリー全体の動作を管理するクラス
 * @author okuyama0528
 */

#include "EtRallyMotion.h"

namespace {

  /**
   * @brief 回頭終了判定許容誤差[deg]
   */
  constexpr double ROTATION_TOLERANCE = 2.0;

}  // namespace

EtRallyMotion::EtRallyMotion(Robot& _robot,
                             std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                             const Pid::PidGain& _rotationPid,
                             const Pid::PidGain& _squareRotationPid,
                             const Pid::PidGain& _straightAnglePid, double _targetSpeed,
                             double _straightDeadbandRate, double _straightMaxoutRate)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    rotationPid(_rotationPid),
    squareRotationPid(_squareRotationPid),
    straightAnglePid(_straightAnglePid),
    targetSpeed(_targetSpeed),
    straightDeadbandRate(_straightDeadbandRate),
    straightMaxoutRate(_straightMaxoutRate)
{
  LOG_CREATE("EtRallyMotion");
}

void EtRallyMotion::prepare() {}

void EtRallyMotion::executeStep()
{
  // キャリブレーターで選択されたコースを取得
  const Course course = robot.getCourse();

  // Robotが保持しているマップ情報を取得
  MapData& mapData = robot.getMapDataInstance();
  GateRoutePlanner routePlanner(mapData);
  EtRallyMap etRallyMap;

  // ET相撲終了後のETラリー開始位置（Lコース基準）
  const Point leftCourseStartPoint{ 10, 2 };

  // マップの範囲外で経路探索を実行しないようにする
  if(leftCourseStartPoint.x < 0 || leftCourseStartPoint.x > SystemInfo::X_GRID_NUM
     || leftCourseStartPoint.y < 0 || leftCourseStartPoint.y > SystemInfo::Y_GRID_NUM) {
    Logger::error("EtRallyMotion: start point is outside the rally map");
    robot.getWheelMotorControllerInstance().stopBoth();
    return;
  }

  // Rコースの場合だけ座標を左右反転
  const Point startPoint = convertPoint(leftCourseStartPoint, course);
  int currentGridX = startPoint.x;
  int currentGridY = startPoint.y;

  // Rコースの場合だけ方向を左右反転
  Direction currentDirection = convertDirection(Direction::RIGHT, course);

  // 通過するゲートの順番
  constexpr GoalColor TARGET_COLORS[] = { GoalColor::RED, GoalColor::BLUE, GoalColor::YELLOW };

  // ETラリー開始位置で正方形を検出して補正する
  Logger::info("ETラリー開始位置QR補正");

  CameraServer::SquareDetectorRequest request{};
  request.roi.x = 0;
  request.roi.y = 0;
  request.roi.width = CAM_MAX_WIDTH;
  request.roi.height = CAM_MAX_HEIGHT;

  SquareAngleAdjustment adjustment(robot);
  SquareAngleAdjustment::Result result = adjustment.calculate(request);

  if(result.wasDetected) {
    Logger::printfLog(Logger::INFO, "Start square detected: angle=%f distance=%f",
                      result.correctionAngle, result.straightDistance);

    // 正方形の方向へ回頭
    auto rotateCondition = std::make_unique<RelativeAngleCondition>(robot, result.correctionAngle,
                                                                    ROTATION_TOLERANCE);
    RelativeRotation rotate(robot, std::move(rotateCondition), squareRotationPid,
                            result.correctionAngle);
    rotate.run();

    // 正方形まで直進
    auto straightCondition = std::make_unique<DistanceCondition>(robot, result.straightDistance);
    Straight straight(robot, std::move(straightCondition), targetSpeed, straightAnglePid, true,
                      straightDeadbandRate, straightMaxoutRate);
    straight.run();

    // 元のETラリー開始方向へ戻す
    auto returnCondition = std::make_unique<RelativeAngleCondition>(robot, -result.correctionAngle,
                                                                    ROTATION_TOLERANCE);
    RelativeRotation returnRotate(robot, std::move(returnCondition), squareRotationPid,
                                  -result.correctionAngle);
    returnRotate.run();
    Logger::info("ETラリー開始位置QR補正完了");

  } else {
    // QRを検出できなければ補正を行わずに処理を続行
    Logger::info("ETラリー開始位置QR補正失敗: QRを検出できませんでした");
  }

  // RouteFollower生成
  RouteFollower routeFollower(robot, etRallyMap, targetSpeed, rotationPid, squareRotationPid,
                              straightAnglePid, straightDeadbandRate, straightMaxoutRate);

  // ゲート情報の確認
  // 必要なゲート情報が不足している場合は停止
  if(!hasAllGates(mapData)) {
    Logger::error("ゲート情報が不足しています");
    robot.getWheelMotorControllerInstance().stopBoth();
    return;
  }

  // ETラリー周回
  int lap = 1;

  while(true) {
    for(const GoalColor targetColor : TARGET_COLORS) {
      Logger::printfLog(Logger::INFO, "ラップ %d: %s ゲートへ移動", lap,
                        colorToString(targetColor));

      // 現在位置と方向から指定色のゲートまで経路探索
      GateRouteResult routeResult
          = routePlanner.search(currentGridX, currentGridY, currentDirection, targetColor);

      if(routeResult.route.size() < 2) {
        Logger::error("EtRallyMotion: route search failed");
        robot.getWheelMotorControllerInstance().stopBoth();
        return;
      }

      // 目標ゲートを取得
      const Gate* targetGate = findGate(mapData, targetColor);

      if(targetGate == nullptr) {
        Logger::error("EtRallyMotion: target gate not found");
        robot.getWheelMotorControllerInstance().stopBoth();
        return;
      }

      // 外周ゲートかどうか判定
      const bool outerGate = routeFollower.isOuterGate(*targetGate);

      // 探索した経路をRouteFollowerに設定して走行
      routeFollower.setRoute(routeResult.route);
      routeFollower.run();

      // 通過後の現在位置と方向を更新
      if(outerGate) {
        // 外周ゲートでは入口側へ戻る
        currentGridX = routeResult.entrance.x;
        currentGridY = routeResult.entrance.y;
        currentDirection = routeResult.exitDirection;

      } else {
        // 内側ゲートでは出口側へ進む
        currentGridX = routeResult.exit.x;
        currentGridY = routeResult.exit.y;
        currentDirection = routeResult.exitDirection;
      }
    }

    ++lap;
  }
}

void EtRallyMotion::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}

Point EtRallyMotion::convertPoint(const Point& point, Course course)
{
  if(course == Course::Right) {
    return CourseMirror::mirrorPoint(point);
  }
  return point;
}

Direction EtRallyMotion::convertDirection(Direction direction, Course course)
{
  if(course == Course::Right) {
    return CourseMirror::mirrorDirection(direction);
  }
  return direction;
}

const char* EtRallyMotion::colorToString(GoalColor color)
{
  switch(color) {
    case GoalColor::RED:
      return "RED";

    case GoalColor::BLUE:
      return "BLUE";

    case GoalColor::YELLOW:
      return "YELLOW";
  }

  return "UNKNOWN";
}

const Gate* EtRallyMotion::findGate(const MapData& mapData, GoalColor color)
{
  for(const Gate& gate : mapData.getGates()) {
    if(gate.color == color) {
      return &gate;
    }
  }

  return nullptr;
}

bool EtRallyMotion::hasAllGates(const MapData& mapData)
{
  return findGate(mapData, GoalColor::RED) != nullptr
         && findGate(mapData, GoalColor::BLUE) != nullptr
         && findGate(mapData, GoalColor::YELLOW) != nullptr;
}