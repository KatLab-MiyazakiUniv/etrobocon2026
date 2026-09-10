/**
 * @file   EtRobocon2026.cpp
 * @brief  L/Rコース対応で3色のゲートをRouteFollowerで走行するテスト
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
  enum class CourseSide { L_COURSE, R_COURSE };

  /**
   * @brief 使用するコース
   */
  constexpr CourseSide COURSE_SIDE = CourseSide::L_COURSE;

  /**
   * @brief Lコース座標を使用コース用へ変換する
   */
  Point convertPoint(const Point& point)
  {
    if(COURSE_SIDE == CourseSide::R_COURSE) {
      return CourseMirror::mirrorPoint(point);
    }

    return point;
  }

  /**
   * @brief Lコース方向を使用コース用へ変換する
   */
  Direction convertDirection(Direction direction)
  {
    if(COURSE_SIDE == CourseSide::R_COURSE) {
      return CourseMirror::mirrorDirection(direction);
    }

    return direction;
  }

  /**
   * @brief Directionを文字列化する
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
   * @brief GoalColorを文字列化する
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
  Logger::info("RouteFollower Square gate correction test start");

  // =========================================================
  // 1. Robot
  // =========================================================

  RealNetworkSystem networkSystem;

  SocketClient cameraSocketClient(networkSystem);

  Robot robot(cameraSocketClient);

  // =========================================================
  // カメラサーバー接続
  //
  // ゲート手前での正方形検出・角度補正に使用する。
  // =========================================================

  Logger::info("EtRobocon2026: connect to camera server");

  robot.getCameraSocketClientInstance().connectToServer();

  // =========================================================
  // 2. ゲート情報
  // =========================================================

  MapData mapData;

  // ---------------------------------------------------------
  // 赤ゲート
  // ---------------------------------------------------------

  Point redGate1 = convertPoint({ 1, 5 });

  Point redGate2 = convertPoint({ 3, 5 });

  mapData.setGate(GoalColor::RED, redGate1, redGate2);

  // ---------------------------------------------------------
  // 青ゲート
  // ---------------------------------------------------------

  Point blueGate1 = convertPoint({ 5, 7 });

  Point blueGate2 = convertPoint({ 5, 9 });

  mapData.setGate(GoalColor::BLUE, blueGate1, blueGate2);

  // ---------------------------------------------------------
  // 黄ゲート
  // ---------------------------------------------------------

  Point yellowGate1 = convertPoint({ 7, 5 });

  Point yellowGate2 = convertPoint({ 9, 5 });

  mapData.setGate(GoalColor::YELLOW, yellowGate1, yellowGate2);

  // =========================================================
  // 3. 経路探索
  // =========================================================

  GateRoutePlanner routePlanner(mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 4. 開始位置
  // =========================================================

  Point startPoint = convertPoint({ 2, 2 });

  int currentGridX = startPoint.x;

  int currentGridY = startPoint.y;

  Direction currentDirection = convertDirection(Direction::DOWN);

  Logger::printfLog(Logger::INFO, "Start grid: (%d,%d), direction=%s", currentGridX, currentGridY,
                    directionToString(currentDirection));

  // =========================================================
  // 5. 通過ゲート
  // =========================================================

  constexpr GoalColor TARGET_COLORS[] = { GoalColor::RED, GoalColor::BLUE, GoalColor::YELLOW };

  // =========================================================
  // 6. PID
  // =========================================================

  /**
   * @brief 回頭PID
   */
  const Pid::PidGain rotationPid = { 1.3, 1.0, 0.0 };

  /**
   * @brief 右車輪速度PID
   */
  const Pid::PidGain rightPid = { 0.016, 0.005, 0.0015 };

  /**
   * @brief 左車輪速度PID
   */
  const Pid::PidGain leftPid = { 0.016, 0.0045, 0.0015 };

  /**
   * @brief Straight角度PID
   */
  const Pid::PidGain straightAnglePid = { 0.033, 0.003, 0.03 };

  /**
   * @brief 正方形中心補正PID
   *
   * 内側ゲートを通過するときに、
   * SquareAngleAdjustmentで使用する。
   *
   * 正方形の中心が画像中央に来るように
   * ロボットの向きを調整する。
   */
  const Pid::PidGain squareAnglePid = { 3.0, 0.0, 0.1 };

  // =========================================================
  // 7. 走行速度
  // =========================================================

  constexpr double TARGET_SPEED = 200.0;

  // =========================================================
  // 8. RouteFollower
  //
  // MapDataも渡すことで、
  // RouteFollower内部で現在の区間が
  // ゲート通過区間か判定できるようにする。
  // =========================================================

  RouteFollower routeFollower(robot, etRallyMap, mapData, TARGET_SPEED, rotationPid, rightPid,
                              leftPid, straightAnglePid, squareAnglePid);

  // =========================================================
  // 9. RED → BLUE → YELLOW × 3
  // =========================================================

  constexpr int LAP_COUNT = 3;

  for(int lap = 1; lap <= LAP_COUNT; ++lap) {
    Logger::info("========================================");

    Logger::printfLog(Logger::INFO, "Lap %d / %d start", lap, LAP_COUNT);

    for(const GoalColor targetColor : TARGET_COLORS) {
      Logger::info("==============================");

      Logger::printfLog(Logger::INFO, "Lap %d / %d Target=%s", lap, LAP_COUNT,
                        colorToString(targetColor));

      // =====================================================
      // 経路探索
      // =====================================================

      GateRouteResult routeResult
          = routePlanner.search(currentGridX, currentGridY, currentDirection, targetColor);

      if(!routeResult.found) {
        Logger::printfLog(Logger::ERROR, "Route to %s not found", colorToString(targetColor));

        robot.getWheelMotorControllerInstance().stopBoth();

        return;
      }

      // =====================================================
      // 経路表示
      // =====================================================

      Logger::printfLog(Logger::INFO, "Route cost=%d", routeResult.cost);

      Logger::printfLog(Logger::INFO, "Route size=%d", static_cast<int>(routeResult.route.size()));

      for(std::size_t i = 0; i < routeResult.route.size(); ++i) {
        const RouteState& state = routeResult.route[i];

        const EtRallyMap::Node node = etRallyMap.getNode(state.x, state.y);

        Logger::printfLog(Logger::INFO,
                          "Route[%d]: "
                          "grid=(%d,%d) "
                          "position=(%.2f,%.2f) "
                          "direction=%s",
                          static_cast<int>(i), state.x, state.y, node.x, node.y,
                          directionToString(state.direction));
      }

      // =====================================================
      // RouteFollower
      //
      // 通常区間:
      //
      // Rotation
      // ↓
      // Straight
      //
      //
      // 内側ゲート区間:
      //
      // Rotation
      // ↓
      // Straight
      // ↓
      // ゲート375mm手前
      // ↓
      // SquareAngleAdjustment 1回目
      // ↓
      // Straight 250mm
      // ↓
      // ゲート125mm手前
      // ↓
      // SquareAngleAdjustment 2回目
      // ↓
      // Straight
      // ↓
      // ゲート通過
      //
      //
      // 外周ゲート:
      //
      // SquareAngleAdjustmentを使用せず
      // Straightで通過する。
      //
      //
      // SquareAngleAdjustmentで
      // 正方形を検出できなかった場合も、
      // 走行を中止せずStraightを継続する。
      //
      // 各制御の切り替え前に
      // stop + 200ms sleep
      // =====================================================

      routeFollower.run(routeResult.route);

      // =====================================================
      // 現在状態更新
      // =====================================================

      currentGridX = routeResult.exit.x;

      currentGridY = routeResult.exit.y;

      currentDirection = routeResult.exitDirection;

      Logger::printfLog(Logger::INFO,
                        "%s gate passed: "
                        "(%d,%d) %s",
                        colorToString(targetColor), currentGridX, currentGridY,
                        directionToString(currentDirection));
    }

    Logger::printfLog(Logger::INFO, "Lap %d finished", lap);
  }

  // =========================================================
  // 10. 最終地点
  // =========================================================

  Point finalPoint = convertPoint({ 8, 0 });

  Direction finalDirection = convertDirection(Direction::LEFT);

  // =========================================================
  // 11. 最終地点への経路探索
  // =========================================================

  DijkstraRoutePlanner finalRoutePlanner(mapData.getGates());

  RouteResult finalRoute = finalRoutePlanner.search(currentGridX, currentGridY, currentDirection,
                                                    finalPoint, finalDirection);

  if(!finalRoute.found) {
    Logger::error("Route to final position not found");

    robot.getWheelMotorControllerInstance().stopBoth();

    return;
  }

  // =========================================================
  // 12. 最終走行
  //
  // 通常経路としてRouteFollowerで走行する。
  //
  // 途中に登録済みゲート区間が含まれる場合は、
  // RouteFollower側のゲート判定に従って処理する。
  // =========================================================

  routeFollower.run(finalRoute.route);

  // =========================================================
  // 13. 停止
  // =========================================================

  robot.getWheelMotorControllerInstance().stopBoth();

  Logger::printfLog(Logger::INFO, "Final grid=(%d,%d) direction=%s", finalPoint.x, finalPoint.y,
                    directionToString(finalDirection));

  Logger::info("RouteFollower Square gate correction test finished");
}
