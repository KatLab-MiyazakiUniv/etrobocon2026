
/**
 * @file   EtRallyMotion.cpp
 * @brief  ETラリーの経路探索・継続周回・QR補正
 * @author okuyama0528
 */

#include "EtRallyMotion.h"

namespace {

  /**
   * @brief 回頭終了判定許容誤差[deg]
   * @note RouteFollowerと同じ値を使用する
   */
  constexpr double ROTATION_TOLERANCE = 2.0;

  /**
   * @brief Lコース座標を現在のコースに応じて変換する
   * @param point Lコース基準の座標
   * @param course 現在のコース
   * @return 変換後の座標
   */
  Point convertPoint(const Point& point, Course course)
  {
    if(course == Course::Right) {
      return CourseMirror::mirrorPoint(point);
    }
    return point;
  }

  /**
   * @brief Lコース方向を現在のコースに応じて変換する
   * @param direction Lコース基準の方向
   * @param course 現在のコース
   * @return 変換後の方向
   */
  Direction convertDirection(Direction direction, Course course)
  {
    if(course == Course::Right) {
      return CourseMirror::mirrorDirection(direction);
    }
    return direction;
  }

  /**
   * @brief GoalColorを文字列へ変換する
   * @param color ゲートの色
   * @return ゲート色を表す文字列
   */
  const char* colorToString(GoalColor color)
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

  /**
   * @brief 外周ゲートかどうか判定する
   * @param gate 判定するゲート
   * @return true 外周ゲート
   * @return false 内側ゲート
   */
  bool isOuterGate(const Gate& gate)
  {
    // 横向きゲート
    if(gate.start.y == gate.end.y) {
      const int gateY = gate.start.y;
      return gateY == 1 || gateY == SystemInfo::Y_GRID_NUM - 1;
    }

    // 縦向きゲート
    if(gate.start.x == gate.end.x) {
      const int gateX = gate.start.x;
      return gateX == 1 || gateX == SystemInfo::X_GRID_NUM - 1;
    }

    return false;
  }

  /**
   * @brief 指定した色のゲートを取得する
   * @param mapData マップ情報
   * @param color 取得するゲート色
   * @return 指定色のゲートへのポインタ
   */
  const Gate* findGate(const MapData& mapData, GoalColor color)
  {
    for(const Gate& gate : mapData.getGates()) {
      if(gate.color == color) {
        return &gate;
      }
    }

    return nullptr;
  }

  /**
   * @brief 必要な3色のゲート情報が存在するか判定する
   * @param mapData マップ情報
   * @return true 3色すべて存在する
   * @return false 1色以上不足している
   */
  bool hasAllGates(const MapData& mapData)
  {
    return findGate(mapData, GoalColor::RED) != nullptr
           && findGate(mapData, GoalColor::BLUE) != nullptr
           && findGate(mapData, GoalColor::YELLOW) != nullptr;
  }

}  // namespace

EtRallyMotion::EtRallyMotion(
    Robot& _robot,
    std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    const Pid::PidGain& _rotationPid,
    const Pid::PidGain& _squareRotationPid,
    const Pid::PidGain& _straightAnglePid,
    double _targetSpeed,
    double _straightDeadbandRate,
    double _straightMaxoutRate)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    rotationPid(_rotationPid),
    squareRotationPid(_squareRotationPid),
    straightAnglePid(_straightAnglePid),
    targetSpeed(_targetSpeed),
    straightDeadbandRate(_straightDeadbandRate),
    straightMaxoutRate(_straightMaxoutRate)
{
}

void EtRallyMotion::prepare() {}

void EtRallyMotion::executeStep()
{
  Logger::info("Timed ET Rally start");

  // ========================================
  // ETラリー準備
  // ========================================

  // キャリブレーターで選択されたコースを取得
  const Course course = robot.getCourse();

  Logger::printfLog(Logger::INFO, "ET Rally course: %s",
                    course == Course::Right ? "Right" : "Left");

  // Robotが保持しているマップ情報を取得
  MapData& mapData = robot.getMapDataInstance();

  GateRoutePlanner routePlanner(mapData);
  EtRallyMap etRallyMap;

  // ET相撲終了後のETラリー開始位置（Lコース基準）
  const Point leftCourseStartPoint{ 10, 2 };

  // マップの範囲外で経路探索を実行しないようにする
  // 格子座標は0〜X_GRID_NUM、0〜Y_GRID_NUM
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

  // Lコースでは右向き、Rコースでは左向き
  Direction currentDirection = convertDirection(Direction::RIGHT, course);

  // 通過するゲートの順番
  constexpr GoalColor TARGET_COLORS[] = {
    GoalColor::RED,
    GoalColor::BLUE,
    GoalColor::YELLOW
  };

  // PIDゲイン、走行速度などはCSVからMotionParser経由で設定済み

  // ========================================
  // ETラリー開始位置QR補正
  // ========================================

  Logger::info("ET Rally start square adjustment");

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

    // ----------------------------------------
    // QR方向へ回頭
    // ----------------------------------------

    auto rotateCondition = std::make_unique<RelativeAngleCondition>(
        robot, result.correctionAngle, ROTATION_TOLERANCE);

    RelativeRotation rotate(robot, std::move(rotateCondition), squareRotationPid,
                            result.correctionAngle);

    rotate.run();

    // ----------------------------------------
    // QRまで直進
    // ----------------------------------------

    auto straightCondition = std::make_unique<DistanceCondition>(
        robot, result.straightDistance);

    Straight straight(robot, std::move(straightCondition), targetSpeed,
                      straightAnglePid, true, straightDeadbandRate, straightMaxoutRate);

    straight.run();

    // ----------------------------------------
    // 元のETラリー開始方向へ戻す
    // ----------------------------------------

    auto returnCondition = std::make_unique<RelativeAngleCondition>(
        robot, -result.correctionAngle, ROTATION_TOLERANCE);

    RelativeRotation returnRotate(robot, std::move(returnCondition), squareRotationPid,
                                  -result.correctionAngle);

    returnRotate.run();

    Logger::info("ET Rally start square adjustment finished");

  } else {

    // QRを検出できなければ補正を行わずに処理を続行
    Logger::info("ET Rally start square not detected -> skip adjustment");
  }

  // ========================================
  // RouteFollower生成
  // ========================================

  RouteFollower routeFollower(robot, etRallyMap, targetSpeed, rotationPid, squareRotationPid,
                              straightAnglePid, straightDeadbandRate, straightMaxoutRate);

  // ========================================
  // ゲート情報の確認
  // ========================================

  // 必要なゲート情報が不足している場合は停止
  if(!hasAllGates(mapData)) {
    Logger::error("EtRallyMotion: gate information is incomplete");
    robot.getWheelMotorControllerInstance().stopBoth();
    return;
  }

  // ========================================
  // ETラリーの継続周回
  // ========================================

  // 赤 → 青 → 黄 → 赤 → ... の順に繰り返す
  // 経路探索に失敗した場合は停止する

  int lap = 1;

  while(true) {

    for(const GoalColor targetColor : TARGET_COLORS) {

      Logger::printfLog(Logger::INFO, "Lap %d Target=%s",
                        lap, colorToString(targetColor));

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
      const bool outerGate = isOuterGate(*targetGate);

      // 探索した経路をRouteFollowerに設定して走行
      routeFollower.setRoute(routeResult.route);
      routeFollower.run();

      // ========================================
      // 通過後の現在位置と方向を更新
      // ========================================

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
