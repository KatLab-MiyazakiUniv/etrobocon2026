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
  Logger::info("RouteFollower QR gate test start");

  // =========================================================
  // 1. Robot生成
  // =========================================================

  RealNetworkSystem networkSystem;

  SocketClient cameraSocketClient(networkSystem);

  Robot robot(cameraSocketClient);

  /*
   * QRコード検出を使用するため、
   * カメラサーバーへ接続する。
   */
  robot.getCameraSocketClientInstance().connectToServer();

  // =========================================================
  // 2. ゲート情報登録
  // =========================================================

  MapData mapData;

  // ---------------------------------------------------------
  // 赤ゲート
  // ---------------------------------------------------------

  Point redGate1 = convertPoint({ 1, 9 });

  Point redGate2 = convertPoint({ 3, 9 });

  mapData.setGate(GoalColor::RED, redGate1, redGate2);

  // ---------------------------------------------------------
  // 青ゲート
  // ---------------------------------------------------------

  Point blueGate1 = convertPoint({ 3, 5 });

  Point blueGate2 = convertPoint({ 3, 7 });

  mapData.setGate(GoalColor::BLUE, blueGate1, blueGate2);

  // ---------------------------------------------------------
  // 黄ゲート
  // ---------------------------------------------------------

  Point yellowGate1 = convertPoint({ 7, 5 });

  Point yellowGate2 = convertPoint({ 9, 5 });

  mapData.setGate(GoalColor::YELLOW, yellowGate1, yellowGate2);

  // =========================================================
  // 3. 経路探索クラス・実座標マップ
  // =========================================================

  GateRoutePlanner routePlanner(mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 4. 開始位置
  // =========================================================

  Point startPoint = convertPoint({ 0, 4 });

  int currentGridX = startPoint.x;

  int currentGridY = startPoint.y;

  Direction currentDirection = convertDirection(Direction::LEFT);

  Logger::printfLog(Logger::INFO, "Start grid: (%d,%d), direction=%s", currentGridX, currentGridY,
                    directionToString(currentDirection));

  // =========================================================
  // 5. 通過するゲート
  // =========================================================

  constexpr GoalColor TARGET_COLORS[] = { GoalColor::RED, GoalColor::BLUE, GoalColor::YELLOW };

  // =========================================================
  // 6. PID設定
  // =========================================================

  /*
   * 回頭PID
   */
  const Pid::PidGain rotationPid = { 1.3, 1.0, 0.0 };

  /*
   * 右車輪速度PID
   */
  const Pid::PidGain rightPid = { 0.016, 0.005, 0.0015 };

  /*
   * 左車輪速度PID
   */
  const Pid::PidGain leftPid = { 0.016, 0.0045, 0.0015 };

  /*
   * 通常Straight用角度補正PID
   */
  const Pid::PidGain straightAnglePid = { 0.033, 0.003, 0.03 };

  /*
   * QRコード追従用PID
   *
   * QRTracking設定:
   *
   * speed   = 350.0
   * targetX = 960
   * kp      = 0.000161
   * ki      = 0.00001
   * kd      = 0.0001019
   */
  const Pid::PidGain qrTrackingPid = { 0.000161, 0.00001, 0.0001019 };

  /*
   * 通常走行速度
   */
  constexpr double TARGET_SPEED = 400.0;

  /*
   * QR追従時の速度
   */
  constexpr double QR_TRACKING_SPEED = 350.0;

  /*
   * QRコードを画像の中央へ合わせるための目標X座標。
   *
   * 1920px幅の画像なので中央は960。
   */
  constexpr int QR_TARGET_X = 960;

  // =========================================================
  // 7. RouteFollower生成
  // =========================================================

  RouteFollower routeFollower(robot, etRallyMap, mapData, TARGET_SPEED, QR_TRACKING_SPEED,
                              QR_TARGET_X, rotationPid, rightPid, leftPid, straightAnglePid,
                              qrTrackingPid);

  // =========================================================
  // 8. RED → BLUE → YELLOW を3周
  // =========================================================

  constexpr int LAP_COUNT = 3;

  for(int lap = 1; lap <= LAP_COUNT; ++lap) {
    Logger::info("========================================");

    Logger::printfLog(Logger::INFO, "Lap %d / %d start", lap, LAP_COUNT);

    for(const GoalColor targetColor : TARGET_COLORS) {
      Logger::info("==============================");

      Logger::printfLog(Logger::INFO, "Lap %d / %d, Target gate: %s", lap, LAP_COUNT,
                        colorToString(targetColor));

      Logger::printfLog(Logger::INFO, "Current grid: (%d,%d), direction=%s", currentGridX,
                        currentGridY, directionToString(currentDirection));

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
      // 探索結果表示
      // =====================================================

      Logger::printfLog(Logger::INFO, "Route cost: %d", routeResult.cost);

      Logger::printfLog(Logger::INFO, "Gate entrance: (%d,%d)", routeResult.entrance.x,
                        routeResult.entrance.y);

      Logger::printfLog(Logger::INFO, "Gate exit: (%d,%d)", routeResult.exit.x, routeResult.exit.y);

      Logger::printfLog(Logger::INFO, "Exit direction: %s",
                        directionToString(routeResult.exitDirection));

      // =====================================================
      // 経路表示
      // =====================================================

      for(std::size_t i = 0; i < routeResult.route.size(); ++i) {
        const RouteState& state = routeResult.route[i];

        const EtRallyMap::Node node = etRallyMap.getNode(state.x, state.y);

        Logger::printfLog(Logger::INFO,
                          "Route[%d]: grid=(%d,%d), "
                          "position=(%.2f, %.2f), "
                          "direction=%s",
                          static_cast<int>(i), state.x, state.y, node.x, node.y,
                          directionToString(state.direction));
      }

      // =====================================================
      // RouteFollowerでゲートまで走行
      //
      // true:
      // QR補正を使用する。
      //
      // 内側ゲート:
      //   ゲート125mm手前までStraight
      //   ↓
      //   QRコード確認
      //   ↓
      //   成功:
      //     QRTracking
      //     speed   = 350
      //     targetX = 960
      //     PID     = QR専用PID
      //     distance = 250mm
      //
      //   失敗:
      //     Straight 250mm
      //   ↓
      //   残りStraight
      //
      // 外周ゲート:
      //   通常Straight
      // =====================================================

      routeFollower.run(routeResult.route, true);

      // =====================================================
      // 次回探索開始位置更新
      // =====================================================

      currentGridX = routeResult.exit.x;

      currentGridY = routeResult.exit.y;

      currentDirection = routeResult.exitDirection;

      Logger::printfLog(Logger::INFO, "%s gate passed: exit=(%d,%d), direction=%s",
                        colorToString(targetColor), currentGridX, currentGridY,
                        directionToString(currentDirection));
    }

    Logger::printfLog(Logger::INFO, "Lap %d / %d finished", lap, LAP_COUNT);
  }

  // =========================================================
  // 9. 最終地点
  // =========================================================

  Logger::info("========================================");

  Logger::info("Move to final position");

  Point finalPoint = convertPoint({ 8, 0 });

  Direction finalDirection = convertDirection(Direction::LEFT);

  // =========================================================
  // 10. 最終地点への経路探索
  // =========================================================

  DijkstraRoutePlanner finalRoutePlanner(mapData.getGates());

  RouteResult finalRoute = finalRoutePlanner.search(currentGridX, currentGridY, currentDirection,
                                                    finalPoint, finalDirection);

  if(!finalRoute.found) {
    Logger::printfLog(Logger::ERROR, "Route to final position (%d,%d) not found", finalPoint.x,
                      finalPoint.y);

    robot.getWheelMotorControllerInstance().stopBoth();

    return;
  }

  // =========================================================
  // 11. 最終地点まで通常走行
  //
  // QR補正は使用しない。
  // =========================================================

  routeFollower.run(finalRoute.route);

  // =========================================================
  // 12. 停止
  // =========================================================

  robot.getWheelMotorControllerInstance().stopBoth();

  Logger::printfLog(Logger::INFO, "Final grid: (%d,%d), direction=%s", finalPoint.x, finalPoint.y,
                    directionToString(finalDirection));

  Logger::info("RouteFollower QR gate test finished");
}
