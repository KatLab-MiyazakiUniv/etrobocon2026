/**
 * @file   EtRobocon2026.cpp
 * @brief  L/Rコース対応で3色のゲートをRouteFollowerで3周走行するテスト
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

#include "CourseMirror.h"
#include "DijkstraRoutePlanner.h"
#include "EtRallyMap.h"
#include "GateRoutePlanner.h"
#include "Logger.h"
#include "MapData.h"
#include "Pid.h"
#include "RealNetworkSystem.h"
#include "Robot.h"
#include "RouteFollower.h"
#include "RouteTypes.h"
#include "SocketClient.h"

namespace {

  /**
   * @brief コースの種類
   */
  enum class CourseSide {
    L_COURSE,
    R_COURSE
  };

  /**
   * @brief 使用するコース
   */
  constexpr CourseSide COURSE_SIDE = CourseSide::L_COURSE;

  /**
   * @brief Lコース基準の座標を使用するコース用へ変換する
   */
  Point convertPoint(const Point& point)
  {
    if(COURSE_SIDE == CourseSide::R_COURSE) {
      return CourseMirror::mirrorPoint(point);
    }

    return point;
  }

  /**
   * @brief Lコース基準の方向を使用するコース用へ変換する
   */
  Direction convertDirection(Direction direction)
  {
    if(COURSE_SIDE == CourseSide::R_COURSE) {
      return CourseMirror::mirrorDirection(direction);
    }

    return direction;
  }

  /**
   * @brief Directionを文字列へ変換する
   */
  const char* directionToString(Direction direction)
  {
    switch(direction) {
      case Direction::UP:
        return "UP";

      case Direction::RIGHT:
        return "RIGHT";

      case Direction::DOWN:
        return "DOWN";

      case Direction::LEFT:
        return "LEFT";
    }

    return "UNKNOWN";
  }

  /**
   * @brief GoalColorを文字列へ変換する
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

}  // namespace

void EtRobocon2026::start()
{
  Logger::info("L/R Three gate RouteFollower test start");

  // =========================================================
  // 1. Robot生成
  // =========================================================

  RealNetworkSystem networkSystem;

  SocketClient cameraSocketClient(networkSystem);

  Robot robot(cameraSocketClient);

  // =========================================================
  // 2. ゲート情報登録
  //
  // Lコース基準
  //
  // L:
  // 10 9 8 7 6 5 4 3 2 1 0
  //
  // R:
  // 0 1 2 3 4 5 6 7 8 9 10
  // =========================================================

  MapData mapData;

  // 赤ゲート
  Point redGate1 = convertPoint({ 7, 9 });
  Point redGate2 = convertPoint({ 9, 9 });

  mapData.setGate(
      GoalColor::RED,
      redGate1,
      redGate2);

  // 青ゲート
  Point blueGate1 = convertPoint({ 7, 5 });
  Point blueGate2 = convertPoint({ 7, 7 });

  mapData.setGate(
      GoalColor::BLUE,
      blueGate1,
      blueGate2);

  // 黄ゲート
  Point yellowGate1 = convertPoint({ 1, 5 });
  Point yellowGate2 = convertPoint({ 3, 5 });

  mapData.setGate(
      GoalColor::YELLOW,
      yellowGate1,
      yellowGate2);

  // =========================================================
  // 3. 経路探索・距離変換
  // =========================================================

  GateRoutePlanner routePlanner(mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 4. 開始位置
  //
  // Lコース基準
  // =========================================================

  Point startPoint = convertPoint({ 2, 8 });

  int currentGridX = startPoint.x;
  int currentGridY = startPoint.y;

  Direction currentDirection =
      convertDirection(Direction::LEFT);

  Logger::printfLog(
      Logger::INFO,
      "Start grid: (%d,%d), direction=%s",
      currentGridX,
      currentGridY,
      directionToString(currentDirection));

  // =========================================================
  // 5. 通過するゲートの順番
  // =========================================================

  constexpr GoalColor TARGET_COLORS[] = {
      GoalColor::RED,
      GoalColor::BLUE,
      GoalColor::YELLOW
  };

  // =========================================================
  // 6. PID設定
  // =========================================================

  const Pid::PidGain rotationPid = {
      1.3,
      1.0,
      0.0
  };

  const Pid::PidGain rightPid = {
      0.016,
      0.005,
      0.0015
  };

  const Pid::PidGain leftPid = {
      0.016,
      0.0045,
      0.0015
  };

  const Pid::PidGain straightAnglePid = {
      0.033,
      0.003,
      0.03
  };

  constexpr double TARGET_SPEED = 400.0;

  // =========================================================
  // 7. RouteFollower生成
  // =========================================================

  RouteFollower routeFollower(
      robot,
      etRallyMap,
      TARGET_SPEED,
      rotationPid,
      rightPid,
      leftPid,
      straightAnglePid);

  // =========================================================
  // 8. オドメトリ初期化
  // =========================================================

  WheelMotorController& wheelMotorController =
      robot.getWheelMotorControllerInstance();

  Odometry& odometry =
      robot.getOdometryInstance();

  odometry.initialize(
      wheelMotorController.getLeftCount(),
      wheelMotorController.getRightCount());

  // =========================================================
  // 9. RED → BLUE → YELLOW を3周走行
  // =========================================================

  constexpr int LAP_COUNT = 3;

  for(int lap = 1; lap <= LAP_COUNT; ++lap) {
    Logger::info("========================================");

    Logger::printfLog(
        Logger::INFO,
        "Lap %d / %d start",
        lap,
        LAP_COUNT);

    for(const GoalColor targetColor : TARGET_COLORS) {
      Logger::info("==============================");

      Logger::printfLog(
          Logger::INFO,
          "Lap %d / %d, Target gate: %s",
          lap,
          LAP_COUNT,
          colorToString(targetColor));

      Logger::printfLog(
          Logger::INFO,
          "Current grid: (%d, %d), direction=%s",
          currentGridX,
          currentGridY,
          directionToString(currentDirection));

      // -------------------------------------------------------
      // 経路探索
      // -------------------------------------------------------

      GateRouteResult routeResult =
          routePlanner.search(
              currentGridX,
              currentGridY,
              currentDirection,
              targetColor);

      if(!routeResult.found) {
        Logger::printfLog(
            Logger::ERROR,
            "Route to %s not found",
            colorToString(targetColor));

        robot
            .getWheelMotorControllerInstance()
            .stopBoth();

        return;
      }

      // -------------------------------------------------------
      // 探索結果表示
      // -------------------------------------------------------

      Logger::printfLog(
          Logger::INFO,
          "Route cost: %d",
          routeResult.cost);

      Logger::printfLog(
          Logger::INFO,
          "Gate entrance: (%d, %d)",
          routeResult.entrance.x,
          routeResult.entrance.y);

      Logger::printfLog(
          Logger::INFO,
          "Gate exit: (%d, %d)",
          routeResult.exit.x,
          routeResult.exit.y);

      Logger::printfLog(
          Logger::INFO,
          "Exit direction: %s",
          directionToString(routeResult.exitDirection));

      Logger::printfLog(
          Logger::INFO,
          "Compressed route size: %d",
          static_cast<int>(
              routeResult.route.size()));

      // -------------------------------------------------------
      // 経路表示
      // -------------------------------------------------------

      for(std::size_t i = 0;
          i < routeResult.route.size();
          ++i) {
        const RouteState& state =
            routeResult.route[i];

        const EtRallyMap::Node node =
            etRallyMap.getNode(
                state.x,
                state.y);

        Logger::printfLog(
            Logger::INFO,
            "Route[%d]: grid=(%d,%d), "
            "position=(%.2f, %.2f), "
            "direction=%s",
            static_cast<int>(i),
            state.x,
            state.y,
            node.x,
            node.y,
            directionToString(state.direction));
      }

      // -------------------------------------------------------
      // RouteFollowerで走行
      // -------------------------------------------------------

      routeFollower.run(
          routeResult.route);

      // -------------------------------------------------------
      // 次回探索開始位置を更新
      // -------------------------------------------------------

      currentGridX =
          routeResult.exit.x;

      currentGridY =
          routeResult.exit.y;

      currentDirection =
          routeResult.exitDirection;

      Logger::printfLog(
          Logger::INFO,
          "%s gate passed: exit=(%d,%d), direction=%s",
          colorToString(targetColor),
          currentGridX,
          currentGridY,
          directionToString(currentDirection));
    }

    Logger::printfLog(
        Logger::INFO,
        "Lap %d / %d finished",
        lap,
        LAP_COUNT);
  }

  // =========================================================
  // 10. 3周終了後に最終地点へ移動
  //
  // Lコース基準:
  //   (8,0) LEFT
  //
  // Rコース:
  //   (2,0) RIGHT
  // =========================================================

  Logger::info("========================================");
  Logger::info("Move to final position");

  Point finalPoint =
      convertPoint({ 8, 0 });

  Direction finalDirection =
      convertDirection(Direction::LEFT);

  Logger::printfLog(
      Logger::INFO,
      "Final target: (%d,%d), direction=%s",
      finalPoint.x,
      finalPoint.y,
      directionToString(finalDirection));

  // =========================================================
  // 11. 最終地点への経路探索
  // =========================================================

  DijkstraRoutePlanner finalRoutePlanner(
      mapData.getGates());

  RouteResult finalRoute =
      finalRoutePlanner.search(
          currentGridX,
          currentGridY,
          currentDirection,
          finalPoint,
          finalDirection);

  if(!finalRoute.found) {
    Logger::printfLog(
        Logger::ERROR,
        "Route to final position (%d,%d) not found",
        finalPoint.x,
        finalPoint.y);

    robot
        .getWheelMotorControllerInstance()
        .stopBoth();

    return;
  }

  Logger::printfLog(
      Logger::INFO,
      "Final route cost: %d",
      finalRoute.cost);

  Logger::printfLog(
      Logger::INFO,
      "Final route size: %d",
      static_cast<int>(
          finalRoute.route.size()));

  // =========================================================
  // 12. 最終経路表示
  // =========================================================

  for(std::size_t i = 0;
      i < finalRoute.route.size();
      ++i) {
    const RouteState& state =
        finalRoute.route[i];

    const EtRallyMap::Node node =
        etRallyMap.getNode(
            state.x,
            state.y);

    Logger::printfLog(
        Logger::INFO,
        "FinalRoute[%d]: grid=(%d,%d), "
        "position=(%.2f, %.2f), "
        "direction=%s",
        static_cast<int>(i),
        state.x,
        state.y,
        node.x,
        node.y,
        directionToString(state.direction));
  }

  // =========================================================
  // 13. 最終地点まで走行
  // =========================================================

  routeFollower.run(
      finalRoute.route);

  // =========================================================
  // 14. 最終状態更新
  // =========================================================

  currentGridX =
      finalPoint.x;

  currentGridY =
      finalPoint.y;

  currentDirection =
      finalDirection;

  // =========================================================
  // 15. 停止
  // =========================================================

  robot
      .getWheelMotorControllerInstance()
      .stopBoth();

  Logger::printfLog(
      Logger::INFO,
      "Final grid: (%d,%d), direction=%s",
      currentGridX,
      currentGridY,
      directionToString(currentDirection));

  Logger::info(
      "L/R Three laps RouteFollower test finished");
}