/**
 * @file   EtRobocon2026.cpp
 * @brief  時間判定付きで3色ゲートを最大3周し、最終地点へ向かう
 * @author HaruArima08 yutaro-1214
 */

#include "EtRobocon2026.h"

#include "ClockUtil.h"
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

  enum class CourseSide { L_COURSE, R_COURSE };

  /**
   * @brief 使用コース
   */
  constexpr CourseSide COURSE_SIDE = CourseSide::L_COURSE;

  /**
   * @brief 最大周回数
   */
  constexpr int LAP_COUNT = 3;

  /**
   * @brief 最終地点へ切り替える時間[ms]
   *
   * 100秒。
   *
   * YELLOW通過後だけ確認する。
   */
  constexpr int FINAL_ROUTE_SWITCH_TIME_MS = 100000;

  /**
   * @brief 走行速度[mm/s]
   */
  constexpr double TARGET_SPEED = 200.0;

  Point convertPoint(const Point& point)
  {
    if(COURSE_SIDE == CourseSide::R_COURSE) {
      return CourseMirror::mirrorPoint(point);
    }

    return point;
  }

  Direction convertDirection(Direction direction)
  {
    if(COURSE_SIDE == CourseSide::R_COURSE) {
      return CourseMirror::mirrorDirection(direction);
    }

    return direction;
  }

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
  Logger::info("Timed ET Rally start");

  // =========================================================
  // 1. Robot
  // =========================================================

  RealNetworkSystem networkSystem;

  SocketClient cameraSocketClient(networkSystem);

  Robot robot(cameraSocketClient);

  // =========================================================
  // 2. カメラサーバー
  // =========================================================

  Logger::info("EtRobocon2026: "
               "connect to camera server");

  robot.getCameraSocketClientInstance().connectToServer();

  // =========================================================
  // 3. ゲート
  // =========================================================

  MapData mapData;

  // RED
  mapData.setGate(GoalColor::RED, convertPoint({ 1, 5 }), convertPoint({ 3, 5 }));

  // BLUE
  mapData.setGate(GoalColor::BLUE, convertPoint({ 5, 7 }), convertPoint({ 5, 9 }));

  // YELLOW
  mapData.setGate(GoalColor::YELLOW, convertPoint({ 7, 5 }), convertPoint({ 9, 5 }));

  Logger::printfLog(Logger::INFO, "registered gates=%d",
                    static_cast<int>(mapData.getGates().size()));

  // =========================================================
  // 4. 経路探索
  // =========================================================

  GateRoutePlanner routePlanner(mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 5. 開始位置
  // =========================================================

  const Point startPoint = convertPoint({ 2, 2 });

  int currentGridX = startPoint.x;

  int currentGridY = startPoint.y;

  Direction currentDirection = convertDirection(Direction::DOWN);

  // =========================================================
  // 6. ゲート順
  // =========================================================

  constexpr GoalColor TARGET_COLORS[] = { GoalColor::RED, GoalColor::BLUE, GoalColor::YELLOW };

  // =========================================================
  // 7. PID
  // =========================================================

  const Pid::PidGain rotationPid = { 1.3, 1.0, 0.0 };

  const Pid::PidGain rightPid = { 0.016, 0.005, 0.0015 };

  const Pid::PidGain leftPid = { 0.016, 0.0045, 0.0015 };

  const Pid::PidGain straightAnglePid = { 0.033, 0.003, 0.03 };

  // =========================================================
  // 8. RouteFollower
  // =========================================================

  RouteFollower routeFollower(robot, etRallyMap, mapData, TARGET_SPEED, rotationPid, rightPid,
                              leftPid, straightAnglePid);

  // =========================================================
  // 9. タイマー開始
  // =========================================================

  const int startTime = ClockUtil::now();

  bool shouldGoFinal = false;

  // =========================================================
  // 10. 最大3周
  // =========================================================

  for(int lap = 1; lap <= LAP_COUNT && !shouldGoFinal; ++lap) {
    Logger::info("========================================");

    Logger::printfLog(Logger::INFO, "Lap %d / %d START", lap, LAP_COUNT);

    for(const GoalColor targetColor : TARGET_COLORS) {
      Logger::printfLog(Logger::INFO, "Lap %d Target=%s", lap, colorToString(targetColor));

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

      Logger::printfLog(Logger::INFO, "Route cost=%d size=%d", routeResult.cost,
                        static_cast<int>(routeResult.route.size()));

      // =====================================================
      // 走行
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

      // =====================================================
      // YELLOW後だけ時間確認
      // =====================================================

      if(targetColor == GoalColor::YELLOW) {
        const int elapsedTime = ClockUtil::now() - startTime;

        Logger::printfLog(Logger::INFO,
                          "Elapsed time=%d ms "
                          "(%.2f sec)",
                          elapsedTime, elapsedTime / 1000.0);

        // -------------------------------------------------
        // 3周完了
        // -------------------------------------------------

        if(lap >= LAP_COUNT) {
          Logger::info("3 laps completed "
                       "-> go final");

          shouldGoFinal = true;

          break;
        }

        // -------------------------------------------------
        // 時間閾値
        // -------------------------------------------------

        if(elapsedTime >= FINAL_ROUTE_SWITCH_TIME_MS) {
          Logger::info("Time threshold reached "
                       "-> go final");

          shouldGoFinal = true;

          break;
        }

        Logger::info("Time remains "
                     "-> continue next lap");
      }
    }
  }

  // =========================================================
  // 11. 最終地点
  // =========================================================

  const Point finalPoint = convertPoint({ 8, 0 });

  const Direction finalDirection = convertDirection(Direction::LEFT);

  Logger::printfLog(Logger::INFO, "Final target=(%d,%d) %s", finalPoint.x, finalPoint.y,
                    directionToString(finalDirection));

  // =========================================================
  // 12. 現在位置から最終地点へ再探索
  // =========================================================

  DijkstraRoutePlanner finalRoutePlanner(mapData.getGates());

  RouteResult finalRoute = finalRoutePlanner.search(currentGridX, currentGridY, currentDirection,
                                                    finalPoint, finalDirection);

  if(!finalRoute.found) {
    Logger::error("Route to final position not found");

    robot.getWheelMotorControllerInstance().stopBoth();

    return;
  }

  Logger::printfLog(Logger::INFO, "Final route cost=%d size=%d", finalRoute.cost,
                    static_cast<int>(finalRoute.route.size()));

  // =========================================================
  // 13. 最終走行
  // =========================================================

  routeFollower.run(finalRoute.route);

  // =========================================================
  // 14. 停止
  // =========================================================

  robot.getWheelMotorControllerInstance().stopBoth();

  const int totalTime = ClockUtil::now() - startTime;

  Logger::printfLog(Logger::INFO,
                    "Total time=%d ms "
                    "(%.2f sec)",
                    totalTime, totalTime / 1000.0);

  Logger::info("Timed ET Rally finished");
}