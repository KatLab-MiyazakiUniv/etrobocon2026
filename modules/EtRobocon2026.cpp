/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

#include <memory>

#include "ClockUtil.h"
#include "CourseMirror.h"
#include "DijkstraRoutePlanner.h"
#include "DistanceCondition.h"
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
#include "Straight.h"
#include "SystemInfo.h"

namespace {

  /**
   * @brief コース
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
   * @brief 最終地点へ向かう時間閾値[ms]
   *
   * 100秒
   */
  constexpr int FINAL_ROUTE_SWITCH_TIME_MS = 100000;

  /**
   * @brief 走行速度[mm/s]
   */
  constexpr double TARGET_SPEED = 300.0;

  /**
   * @brief 最終地点到着後の追加直進距離[mm]
   */
  constexpr double FINAL_STRAIGHT_DISTANCE = 100.0;

  /**
   * @brief 直進時のデッドバンド率
   */
  constexpr double STRAIGHT_DEADBAND_RATE = 0.25;

  /**
   * @brief 直進時の最大出力率
   */
  constexpr double STRAIGHT_MAXOUT_RATE = 0.54;

  /**
   * @brief Lコース座標を現在コース用へ変換
   * @param point Lコース座標
   * @return 現在コース用へ変換した座標
   */
  Point convertPoint(const Point& point)
  {
    if(COURSE_SIDE == CourseSide::R_COURSE) {
      return CourseMirror::mirrorPoint(point);
    }

    return point;
  }

  /**
   * @brief Lコース方向を現在コース用へ変換
   * @param direction Lコース方向
   * @return 現在コース用へ変換した方向
   */
  Direction convertDirection(Direction direction)
  {
    if(COURSE_SIDE == CourseSide::R_COURSE) {
      return CourseMirror::mirrorDirection(direction);
    }

    return direction;
  }

  /**
   * @brief Directionを文字列へ変換
   * @param direction 変換する方向
   * @return Directionを表す文字列
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
   * @brief GoalColorを文字列へ変換
   * @param color 変換するゲート色
   * @return GoalColorを表す文字列
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
   * @brief 外周ゲートか判定する
   * @param gate 判定するゲート
   * @return true 外周ゲート
   * @return false 内側ゲート
   */
  bool isOuterGate(const Gate& gate)
  {
    // 横向きゲート
    if(gate.start.y == gate.end.y) {
      const int gateY = gate.start.y;

      /*
       * 走行可能なグリッド座標は偶数で、
       * ゲートはその間の奇数座標に存在するため、
       * 外周ゲートは1または最大値-1になる。
       */
      return gateY == 1 || gateY == SystemInfo::Y_GRID_NUM - 1;
    }

    // 縦向きゲート
    if(gate.start.x == gate.end.x) {
      const int gateX = gate.start.x;

      /*
       * 走行可能なグリッド座標は偶数で、
       * ゲートはその間の奇数座標に存在するため、
       * 外周ゲートは1または最大値-1になる。
       */
      return gateX == 1 || gateX == SystemInfo::X_GRID_NUM - 1;
    }

    return false;
  }

  /**
   * @brief 指定した色のゲートを取得する
   * @param mapData マップ情報
   * @param color 取得するゲート色
   * @return 指定色のゲートへのポインタ
   * @return nullptr 指定色のゲートが存在しない場合
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
  // 2. カメラサーバー接続
  // =========================================================

  Logger::info("EtRobocon2026: "
               "connect to camera server");

  robot.getCameraSocketClientInstance().connectToServer();

  // =========================================================
  // 3. ゲート登録
  // =========================================================

  // Robotが保持しているMapDataを使用する
  MapData& mapData = robot.getMapData();

  // RED
  mapData.setGate(GoalColor::RED, convertPoint({ 1, 1 }), convertPoint({ 3, 1 }));

  // BLUE
  mapData.setGate(GoalColor::BLUE, convertPoint({ 9, 1 }), convertPoint({ 9, 3 }));

  // YELLOW
  mapData.setGate(GoalColor::YELLOW, convertPoint({ 5, 9 }), convertPoint({ 7, 9 }));

  Logger::printfLog(Logger::INFO, "registered gates=%d",
                    static_cast<int>(mapData.getGates().size()));

  // =========================================================
  // 4. マップ・経路探索
  // =========================================================

  GateRoutePlanner routePlanner(mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 5. 開始状態
  // =========================================================

  const Point startPoint = convertPoint({ 0, 4 });

  int currentGridX = startPoint.x;

  int currentGridY = startPoint.y;

  Direction currentDirection = convertDirection(Direction::LEFT);

  Logger::printfLog(Logger::INFO,
                    "Start grid: "
                    "(%d,%d) %s",
                    currentGridX, currentGridY, directionToString(currentDirection));

  // =========================================================
  // 6. ゲート順
  // =========================================================

  constexpr GoalColor TARGET_COLORS[] = { GoalColor::RED, GoalColor::BLUE, GoalColor::YELLOW };

  // =========================================================
  // 7. PID設定
  // =========================================================

  /**
   * @brief 通常回頭用PID
   *
   * 90度回頭などに使用する。
   */
  const Pid::PidGain rotationPid = { 1.4, 0.0, 0.1 };

  /**
   * @brief 正方形補正回頭用PID
   *
   * QR①への微調整、
   * QR①後の-α、
   * QR②への微調整に使用する。
   */
  const Pid::PidGain squareRotationPid = { 4.9, 0.0, 0.12 };

  /**
   * @brief 右モータ速度PID
   */
  const Pid::PidGain rightPid = { 0.014849, 0.004863, 0.0015 };

  /**
   * @brief 左モータ速度PID
   */
  const Pid::PidGain leftPid = { 0.01574, 0.0045, 0.0015 };

  /**
   * @brief 直進角度PID
   */
  const Pid::PidGain straightAnglePid = { 0.033, 0.003, 0.03 };

  const double straightDeadbandRate = 0.25;
  const double straightMaxoutRate = 0.54;

  // =========================================================
  // 8. RouteFollower
  // =========================================================

  RouteFollower routeFollower(robot, etRallyMap, TARGET_SPEED, rotationPid, squareRotationPid,
                              straightAnglePid, straightDeadbandRate, straightMaxoutRate);
  // =========================================================
  // 9. 開始時刻
  // =========================================================

  const int startTime = ClockUtil::now();

  bool shouldGoFinal = false;

  // =========================================================
  // 10. 最大3周
  // =========================================================

  for(int lap = 1; lap <= LAP_COUNT && !shouldGoFinal; ++lap) {
    Logger::info("========================================");

    Logger::printfLog(Logger::INFO, "Lap %d / %d START", lap, LAP_COUNT);

    // =====================================================
    // RED → BLUE → YELLOW
    // =====================================================

    for(const GoalColor targetColor : TARGET_COLORS) {
      Logger::printfLog(Logger::INFO, "Lap %d Target=%s", lap, colorToString(targetColor));

      Logger::printfLog(Logger::INFO,
                        "Current grid: "
                        "(%d,%d) %s",
                        currentGridX, currentGridY, directionToString(currentDirection));

      // ===================================================
      // 経路探索
      // ===================================================

      GateRouteResult routeResult
          = routePlanner.search(currentGridX, currentGridY, currentDirection, targetColor);

      Logger::printfLog(Logger::INFO, "Route cost=%d size=%d", routeResult.cost,
                        static_cast<int>(routeResult.route.size()));

      if(routeResult.route.size() < 2) {
        Logger::error("EtRobocon2026: "
                      "route search failed");

        robot.getWheelMotorControllerInstance().stopBoth();

        return;
      }

      Logger::printfLog(Logger::INFO,
                        "Gate entrance: "
                        "(%d,%d)",
                        routeResult.entrance.x, routeResult.entrance.y);

      Logger::printfLog(Logger::INFO,
                        "Gate exit: "
                        "(%d,%d)",
                        routeResult.exit.x, routeResult.exit.y);

      Logger::printfLog(Logger::INFO, "Gate direction: %s",
                        directionToString(routeResult.exitDirection));

      // ===================================================
      // 対象ゲート取得
      // ===================================================

      const Gate* targetGate = findGate(mapData, targetColor);

      if(targetGate == nullptr) {
        Logger::printfLog(Logger::ERROR,
                          "EtRobocon2026: "
                          "%s gate not found",
                          colorToString(targetColor));

        robot.getWheelMotorControllerInstance().stopBoth();

        return;
      }

      const bool outerGate = isOuterGate(*targetGate);

      Logger::printfLog(Logger::INFO, "Target gate type: %s", outerGate ? "OUTER" : "INNER");

      // ===================================================
      // 走行
      // ===================================================

      routeFollower.run(routeResult.route);

      // ===================================================
      // 現在状態更新
      // ===================================================

      if(outerGate) {
        /*
         * 外周ゲートでは、
         *
         * QR①補正
         *     ↓
         * QR①位置から400mm前進
         *     ↓
         * ゲート通過
         *     ↓
         * 400mm後退
         *     ↓
         * QR①位置
         *
         * という動作を行う。
         *
         * そのため次回探索では、
         * ゲート通過前のentranceを
         * 現在位置として使用する。
         */
        currentGridX = routeResult.entrance.x;

        currentGridY = routeResult.entrance.y;

        /*
         * 後退時は回頭を行わないため、
         * ロボットが向いている方向は
         * ゲート通過時の方向を維持する。
         */
        currentDirection = routeResult.exitDirection;

        Logger::printfLog(Logger::INFO,
                          "%s outer gate passed and returned: "
                          "(%d,%d) %s",
                          colorToString(targetColor), currentGridX, currentGridY,
                          directionToString(currentDirection));
      } else {
        /*
         * 内側ゲートでは通常通り
         * ゲート通過後のexitを
         * 次回探索開始位置として使用する。
         */
        currentGridX = routeResult.exit.x;

        currentGridY = routeResult.exit.y;

        currentDirection = routeResult.exitDirection;

        Logger::printfLog(Logger::INFO,
                          "%s gate passed: "
                          "(%d,%d) %s",
                          colorToString(targetColor), currentGridX, currentGridY,
                          directionToString(currentDirection));
      }

      // ===================================================
      // YELLOW通過後のみ時間確認
      // ===================================================

      if(targetColor == GoalColor::YELLOW) {
        const int elapsedTime = ClockUtil::now() - startTime;

        Logger::printfLog(Logger::INFO,
                          "Elapsed time=%d ms "
                          "(%.2f sec)",
                          elapsedTime, elapsedTime / 1000.0);

        // 3周終了
        if(lap >= LAP_COUNT) {
          Logger::info("3 laps completed "
                       "-> go final");

          shouldGoFinal = true;

          break;
        }

        // 時間超過
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
  // 12. 最終地点への経路探索
  // =========================================================

  DijkstraRoutePlanner finalRoutePlanner(mapData.getGates());

  RouteResult finalRoute = finalRoutePlanner.search(currentGridX, currentGridY, currentDirection,
                                                    finalPoint, finalDirection);

  if(finalRoute.route.size() < 2) {
    Logger::error("EtRobocon2026: "
                  "final route search failed");

    robot.getWheelMotorControllerInstance().stopBoth();

    return;
  }

  Logger::printfLog(Logger::INFO, "Final route cost=%d size=%d", finalRoute.cost,
                    static_cast<int>(finalRoute.route.size()));

  // =========================================================
  // 13. 最終地点まで走行
  // =========================================================

  routeFollower.run(finalRoute.route);

  // =========================================================
  // 14. 最終地点到着後に100mm直進
  // =========================================================

  Logger::printfLog(Logger::INFO, "Final point reached -> straight %.1f mm",
                    FINAL_STRAIGHT_DISTANCE);

  auto finalStraightCondition = std::make_unique<DistanceCondition>(robot, FINAL_STRAIGHT_DISTANCE);

  Straight finalStraight(robot, std::move(finalStraightCondition), TARGET_SPEED, straightAnglePid,
                         true, straightDeadbandRate, straightMaxoutRate);

  finalStraight.run();

  Logger::printfLog(Logger::INFO, "Final straight %.1f mm completed", FINAL_STRAIGHT_DISTANCE);

  // =========================================================
  // 15. 停止
  // =========================================================

  robot.getWheelMotorControllerInstance().stopBoth();

  const int totalTime = ClockUtil::now() - startTime;

  Logger::printfLog(Logger::INFO,
                    "Total time=%d ms "
                    "(%.2f sec)",
                    totalTime, totalTime / 1000.0);

  Logger::info("Timed ET Rally finished");
}