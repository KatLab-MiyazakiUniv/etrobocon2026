/**
 * @file   EtRobocon2026.cpp
 * @brief  時間判定付きで3色のゲートを周回し、最終地点へ向かう
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

  /**
   * @brief コースの種類
   */
  enum class CourseSide { L_COURSE, R_COURSE };

  /**
   * @brief 使用するコース
   */
  constexpr CourseSide COURSE_SIDE = CourseSide::L_COURSE;

  /**
   * @brief 最大周回数
   */
  constexpr int LAP_COUNT = 3;

  /**
   * @brief 最終地点へ切り替える時間[ms]
   *
   * 例:
   * 100000ms = 100秒
   *
   * YELLOWゲート通過後にこの時間以上なら
   * 次の周回へ行かず最終地点へ向かう。
   */
  constexpr int FINAL_ROUTE_SWITCH_TIME_MS = 100000;

  /**
   * @brief 通常走行速度[mm/s]
   */
  constexpr double TARGET_SPEED = 200.0;

  /**
   * @brief Lコース座標を使用コース用へ変換する
   *
   * @param point Lコース基準座標
   * @return 使用コース用座標
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
   *
   * @param direction Lコース基準方向
   * @return 使用コース用方向
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
   *
   * @param direction 方向
   * @return 文字列
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
   *
   * @param color ゲート色
   * @return 文字列
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
  Logger::info("RouteFollower timed rally start");

  // =========================================================
  // 1. Robot生成
  // =========================================================

  RealNetworkSystem networkSystem;

  SocketClient cameraSocketClient(networkSystem);

  Robot robot(cameraSocketClient);

  // =========================================================
  // 2. カメラサーバー接続
  // =========================================================

  Logger::info("EtRobocon2026: connect to camera server");

  robot.getCameraSocketClientInstance().connectToServer();

  Logger::info("EtRobocon2026: camera server connected");

  // =========================================================
  // 3. ゲート情報登録
  //
  // 座標はLコース基準。
  // Rコースの場合はconvertPoint()で反転する。
  // =========================================================

  MapData mapData;

  // ---------------------------------------------------------
  // RED
  // ---------------------------------------------------------

  Point redGate1 = convertPoint({ 1, 5 });

  Point redGate2 = convertPoint({ 3, 5 });

  mapData.setGate(GoalColor::RED, redGate1, redGate2);

  // ---------------------------------------------------------
  // BLUE
  // ---------------------------------------------------------

  Point blueGate1 = convertPoint({ 5, 7 });

  Point blueGate2 = convertPoint({ 5, 9 });

  mapData.setGate(GoalColor::BLUE, blueGate1, blueGate2);

  // ---------------------------------------------------------
  // YELLOW
  // ---------------------------------------------------------

  Point yellowGate1 = convertPoint({ 7, 5 });

  Point yellowGate2 = convertPoint({ 9, 5 });

  mapData.setGate(GoalColor::YELLOW, yellowGate1, yellowGate2);

  Logger::printfLog(Logger::INFO, "EtRobocon2026: registered gates = %d",
                    static_cast<int>(mapData.getGates().size()));

  // =========================================================
  // 4. 経路探索
  // =========================================================

  GateRoutePlanner routePlanner(mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 5. 開始地点
  // =========================================================

  Point startPoint = convertPoint({ 2, 2 });

  int currentGridX = startPoint.x;

  int currentGridY = startPoint.y;

  Direction currentDirection = convertDirection(Direction::DOWN);

  Logger::printfLog(Logger::INFO, "Start grid: (%d,%d), direction=%s", currentGridX, currentGridY,
                    directionToString(currentDirection));

  // =========================================================
  // 6. ゲート通過順
  //
  // RED
  // ↓
  // BLUE
  // ↓
  // YELLOW
  // =========================================================

  constexpr GoalColor TARGET_COLORS[] = { GoalColor::RED, GoalColor::BLUE, GoalColor::YELLOW };

  // =========================================================
  // 7. PID設定
  // =========================================================

  /**
   * @brief RelativeRotation用PID
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

  // =========================================================
  // 8. RouteFollower生成
  // =========================================================

  RouteFollower routeFollower(robot, etRallyMap, mapData, TARGET_SPEED, rotationPid, rightPid,
                              leftPid, straightAnglePid);

  // =========================================================
  // 9. 走行開始時刻
  //
  // カメラ接続や初期化時間を除外し、
  // ここからの経過時間を競技走行時間として扱う。
  // =========================================================

  const int startTime = ClockUtil::now();

  Logger::printfLog(Logger::INFO, "Rally timer started: %d ms", startTime);

  // =========================================================
  // 10. 最終地点へ切り替えるフラグ
  // =========================================================

  bool shouldGoFinal = false;

  // =========================================================
  // 11. 最大3周
  // =========================================================

  for(int lap = 1; lap <= LAP_COUNT && !shouldGoFinal; ++lap) {
    Logger::info("========================================");

    Logger::printfLog(Logger::INFO, "Lap %d / %d START", lap, LAP_COUNT);

    // =======================================================
    // RED → BLUE → YELLOW
    // =======================================================

    for(const GoalColor targetColor : TARGET_COLORS) {
      Logger::info("----------------------------------------");

      Logger::printfLog(Logger::INFO, "Lap %d Target=%s", lap, colorToString(targetColor));

      // =====================================================
      // 11-1. 現在位置から対象ゲートへの経路探索
      // =====================================================

      GateRouteResult routeResult
          = routePlanner.search(currentGridX, currentGridY, currentDirection, targetColor);

      if(!routeResult.found) {
        Logger::printfLog(Logger::ERROR, "Route to %s not found", colorToString(targetColor));

        robot.getWheelMotorControllerInstance().stopBoth();

        return;
      }

      Logger::printfLog(Logger::INFO, "Route cost=%d", routeResult.cost);

      Logger::printfLog(Logger::INFO, "Route size=%d", static_cast<int>(routeResult.route.size()));

      // =====================================================
      // 経路表示
      // =====================================================

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
      // 11-2. 実際に走行
      // =====================================================

      routeFollower.run(routeResult.route);

      // =====================================================
      // 11-3. 現在状態更新
      //
      // ゲートを通過した後の位置・方向を保存する。
      // =====================================================

      currentGridX = routeResult.exit.x;

      currentGridY = routeResult.exit.y;

      currentDirection = routeResult.exitDirection;

      Logger::printfLog(Logger::INFO,
                        "%s gate passed: "
                        "grid=(%d,%d) direction=%s",
                        colorToString(targetColor), currentGridX, currentGridY,
                        directionToString(currentDirection));

      // =====================================================
      // 11-4. YELLOW通過後だけ時間確認
      // =====================================================

      if(targetColor == GoalColor::YELLOW) {
        const int currentTime = ClockUtil::now();

        const int elapsedTime = currentTime - startTime;

        Logger::printfLog(Logger::INFO,
                          "Elapsed time after YELLOW "
                          "= %d ms (%.2f sec)",
                          elapsedTime, elapsedTime / 1000.0);

        // ===================================================
        // 3周完了した場合
        //
        // 時間に関係なく最終地点へ向かう。
        // ===================================================

        if(lap >= LAP_COUNT) {
          Logger::info("Lap 3 completed "
                       "-> go to final position");

          shouldGoFinal = true;

          break;
        }

        // ===================================================
        // 時間閾値を超えた場合
        //
        // 次のREDへ行かず最終地点へ向かう。
        // ===================================================

        if(elapsedTime >= FINAL_ROUTE_SWITCH_TIME_MS) {
          Logger::printfLog(Logger::INFO,
                            "Time threshold reached "
                            "(%d >= %d ms)",
                            elapsedTime, FINAL_ROUTE_SWITCH_TIME_MS);

          Logger::info("Switch to final route");

          shouldGoFinal = true;

          break;
        }

        // ===================================================
        // まだ時間がある
        // ===================================================

        Logger::printfLog(Logger::INFO,
                          "Continue next lap "
                          "(%d < %d ms)",
                          elapsedTime, FINAL_ROUTE_SWITCH_TIME_MS);
      }
    }

    Logger::printfLog(Logger::INFO, "Lap %d finished", lap);
  }

  // =========================================================
  // 12. 最終地点
  // =========================================================

  Point finalPoint = convertPoint({ 8, 0 });

  Direction finalDirection = convertDirection(Direction::LEFT);

  Logger::info("========================================");

  Logger::printfLog(Logger::INFO,
                    "Final target: "
                    "grid=(%d,%d) direction=%s",
                    finalPoint.x, finalPoint.y, directionToString(finalDirection));

  // =========================================================
  // 13. YELLOW通過後の現在位置から
  //     最終地点へ経路探索
  // =========================================================

  DijkstraRoutePlanner finalRoutePlanner(mapData.getGates());

  RouteResult finalRoute = finalRoutePlanner.search(currentGridX, currentGridY, currentDirection,
                                                    finalPoint, finalDirection);

  if(!finalRoute.found) {
    Logger::error("Route to final position not found");

    robot.getWheelMotorControllerInstance().stopBoth();

    return;
  }

  Logger::printfLog(Logger::INFO, "Final route cost=%d", finalRoute.cost);

  Logger::printfLog(Logger::INFO, "Final route size=%d", static_cast<int>(finalRoute.route.size()));

  // =========================================================
  // 14. 最終経路表示
  // =========================================================

  for(std::size_t i = 0; i < finalRoute.route.size(); ++i) {
    const RouteState& state = finalRoute.route[i];

    const EtRallyMap::Node node = etRallyMap.getNode(state.x, state.y);

    Logger::printfLog(Logger::INFO,
                      "FinalRoute[%d]: "
                      "grid=(%d,%d) "
                      "position=(%.2f,%.2f) "
                      "direction=%s",
                      static_cast<int>(i), state.x, state.y, node.x, node.y,
                      directionToString(state.direction));
  }

  // =========================================================
  // 15. 最終地点へ走行
  // =========================================================

  Logger::info("Final route START");

  routeFollower.run(finalRoute.route);

  Logger::info("Final route FINISHED");

  // =========================================================
  // 16. 最終停止
  // =========================================================

  robot.getWheelMotorControllerInstance().stopBoth();

  // =========================================================
  // 17. 最終時間
  // =========================================================

  const int totalElapsedTime = ClockUtil::now() - startTime;

  Logger::printfLog(Logger::INFO, "Final grid=(%d,%d) direction=%s", finalPoint.x, finalPoint.y,
                    directionToString(finalDirection));

  Logger::printfLog(Logger::INFO,
                    "Total rally time "
                    "= %d ms (%.2f sec)",
                    totalElapsedTime, totalElapsedTime / 1000.0);

  Logger::info("RouteFollower timed rally finished");
}