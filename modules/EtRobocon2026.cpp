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
  enum class CourseSide {
    L_COURSE,
    R_COURSE
  };

  /**
   * @brief 使用するコース
   *
   * Lコース:
   *   CourseSide::L_COURSE
   *
   * Rコース:
   *   CourseSide::R_COURSE
   */
  constexpr CourseSide COURSE_SIDE =
      CourseSide::L_COURSE;

  /**
   * @brief Lコース基準の座標を使用するコース用へ変換する
   * @param point Lコース基準の座標
   * @return 使用するコースの座標
   */
  Point convertPoint(
      const Point& point)
  {
    if(COURSE_SIDE
       == CourseSide::R_COURSE) {

      return CourseMirror::mirrorPoint(
          point);
    }

    return point;
  }

  /**
   * @brief Lコース基準の方向を使用するコース用へ変換する
   * @param direction Lコース基準の方向
   * @return 使用するコースの方向
   */
  Direction convertDirection(
      Direction direction)
  {
    if(COURSE_SIDE
       == CourseSide::R_COURSE) {

      return CourseMirror::mirrorDirection(
          direction);
    }

    return direction;
  }

  /**
   * @brief Directionを文字列へ変換する
   * @param direction 方向
   * @return 方向を表す文字列
   */
  const char* directionToString(
      Direction direction)
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
   * @param color ゲート色
   * @return ゲート色を表す文字列
   */
  const char* colorToString(
      GoalColor color)
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
  Logger::info(
      "RouteFollower Square gate test start");

  // =========================================================
  // 1. Robot生成
  // =========================================================

  RealNetworkSystem networkSystem;

  SocketClient cameraSocketClient(
      networkSystem);

  Robot robot(
      cameraSocketClient);

  /*
   * ゲート通過時に正方形検出を使用するため、
   * カメラサーバーへ接続する。
   */
  Logger::info(
      "EtRobocon2026: connect to camera server");

  robot
      .getCameraSocketClientInstance()
      .connectToServer();

  // =========================================================
  // 2. ゲート情報登録
  //
  // 座標はLコース基準で定義し、
  // Rコースの場合はCourseMirrorで変換する。
  // =========================================================

  MapData mapData;

  // ---------------------------------------------------------
  // 赤ゲート
  // ---------------------------------------------------------

  Point redGate1 =
      convertPoint(
          { 1, 5 });

  Point redGate2 =
      convertPoint(
          { 3, 5 });

  mapData.setGate(
      GoalColor::RED,
      redGate1,
      redGate2);

  // ---------------------------------------------------------
  // 青ゲート
  // ---------------------------------------------------------

  Point blueGate1 =
      convertPoint(
          { 5, 7 });

  Point blueGate2 =
      convertPoint(
          { 5, 9 });

  mapData.setGate(
      GoalColor::BLUE,
      blueGate1,
      blueGate2);

  // ---------------------------------------------------------
  // 黄ゲート
  // ---------------------------------------------------------

  Point yellowGate1 =
      convertPoint(
          { 7, 5 });

  Point yellowGate2 =
      convertPoint(
          { 9, 5 });

  mapData.setGate(
      GoalColor::YELLOW,
      yellowGate1,
      yellowGate2);

  // =========================================================
  // 3. 経路探索クラス・実座標マップ
  // =========================================================

  GateRoutePlanner routePlanner(
      mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 4. 開始位置
  // =========================================================

  Point startPoint =
      convertPoint(
          { 2, 2 });

  int currentGridX =
      startPoint.x;

  int currentGridY =
      startPoint.y;

  Direction currentDirection =
      convertDirection(
          Direction::DOWN);

  Logger::printfLog(
      Logger::INFO,
      "Start grid: (%d,%d), direction=%s",
      currentGridX,
      currentGridY,
      directionToString(
          currentDirection));

  // =========================================================
  // 5. 通過するゲート
  // =========================================================

  constexpr GoalColor TARGET_COLORS[] = {
      GoalColor::RED,
      GoalColor::BLUE,
      GoalColor::YELLOW
  };

  // =========================================================
  // 6. PID設定
  // =========================================================

  /**
   * 回頭PID
   */
  const Pid::PidGain rotationPid = {
      1.3,
      1.0,
      0.0
  };

  /**
   * 右車輪速度PID
   */
  const Pid::PidGain rightPid = {
      0.016,
      0.005,
      0.0015
  };

  /**
   * 左車輪速度PID
   */
  const Pid::PidGain leftPid = {
      0.016,
      0.0045,
      0.0015
  };

  /**
   * 通常Straight用角度補正PID
   */
  const Pid::PidGain straightAnglePid = {
      0.033,
      0.003,
      0.03
  };

  /**
   * 正方形追従用PID
   *
   * 正方形の中心X座標が
   * 画像中央へ来るように補正する。
   */
  const Pid::PidGain squareTrackingPid = {
      0.0010,
      0.000,
      0.0005
  };

  // =========================================================
  // 7. 走行設定
  // =========================================================

  /**
   * @brief 通常Straight速度[mm/s]
   */
  constexpr double TARGET_SPEED =
      200.0;

  /**
   * @brief 正方形追従時の速度[mm/s]
   */
  constexpr double SQUARE_TRACKING_SPEED =
      200.0;

  /**
   * @brief 正方形を合わせる画像上のX座標
   *
   * 1920px幅の画像中央。
   */
  constexpr int SQUARE_TARGET_X =
      960;

  // =========================================================
  // 8. RouteFollower生成
  // =========================================================

  RouteFollower routeFollower(
      robot,
      etRallyMap,
      mapData,
      TARGET_SPEED,
      SQUARE_TRACKING_SPEED,
      SQUARE_TARGET_X,
      rotationPid,
      rightPid,
      leftPid,
      straightAnglePid,
      squareTrackingPid);

  // =========================================================
  // 9. RED → BLUE → YELLOW を3周
  // =========================================================

  constexpr int LAP_COUNT = 3;

  for(int lap = 1;
      lap <= LAP_COUNT;
      ++lap) {

    Logger::info(
        "========================================");

    Logger::printfLog(
        Logger::INFO,
        "Lap %d / %d start",
        lap,
        LAP_COUNT);

    for(const GoalColor targetColor :
        TARGET_COLORS) {

      Logger::info(
          "==============================");

      Logger::printfLog(
          Logger::INFO,
          "Lap %d / %d, Target gate: %s",
          lap,
          LAP_COUNT,
          colorToString(
              targetColor));

      Logger::printfLog(
          Logger::INFO,
          "Current grid: (%d,%d), direction=%s",
          currentGridX,
          currentGridY,
          directionToString(
              currentDirection));

      // =====================================================
      // 経路探索
      // =====================================================

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
            colorToString(
                targetColor));

        robot
            .getWheelMotorControllerInstance()
            .stopBoth();

        return;
      }

      // =====================================================
      // 探索結果表示
      // =====================================================

      Logger::printfLog(
          Logger::INFO,
          "Route cost: %d",
          routeResult.cost);

      Logger::printfLog(
          Logger::INFO,
          "Gate entrance: (%d,%d)",
          routeResult.entrance.x,
          routeResult.entrance.y);

      Logger::printfLog(
          Logger::INFO,
          "Gate exit: (%d,%d)",
          routeResult.exit.x,
          routeResult.exit.y);

      Logger::printfLog(
          Logger::INFO,
          "Exit direction: %s",
          directionToString(
              routeResult.exitDirection));

      Logger::printfLog(
          Logger::INFO,
          "Route size: %d",
          static_cast<int>(
              routeResult.route.size()));

      // =====================================================
      // 経路表示
      // =====================================================

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
            directionToString(
                state.direction));
      }

      // =====================================================
      // RouteFollowerで走行
      //
      // RouteFollower側で各区間を自動判定する。
      //
      // 通常区間:
      //   Straight
      //
      // ゲート通過区間:
      //
      //   Straight
      //      ↓
      //   ゲート125mm手前
      //      ↓
      //   SquareTracking 250mm
      //      ↓
      //   残りStraight
      //
      // true / falseの指定は不要。
      // =====================================================

      Logger::info(
          "EtRobocon2026: RouteFollower start");

      routeFollower.run(
          routeResult.route);

      Logger::info(
          "EtRobocon2026: RouteFollower finished");

      // =====================================================
      // 次回探索開始位置更新
      // =====================================================

      currentGridX =
          routeResult.exit.x;

      currentGridY =
          routeResult.exit.y;

      currentDirection =
          routeResult.exitDirection;

      Logger::printfLog(
          Logger::INFO,
          "%s gate passed: "
          "exit=(%d,%d), direction=%s",
          colorToString(
              targetColor),
          currentGridX,
          currentGridY,
          directionToString(
              currentDirection));
    }

    Logger::printfLog(
        Logger::INFO,
        "Lap %d / %d finished",
        lap,
        LAP_COUNT);
  }

  // =========================================================
  // 10. 最終地点
  // =========================================================

  Logger::info(
      "========================================");

  Logger::info(
      "Move to final position");

  Point finalPoint =
      convertPoint(
          { 8, 0 });

  Direction finalDirection =
      convertDirection(
          Direction::LEFT);

  Logger::printfLog(
      Logger::INFO,
      "Final target: (%d,%d), direction=%s",
      finalPoint.x,
      finalPoint.y,
      directionToString(
          finalDirection));

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

  // =========================================================
  // 12. 最終経路表示
  // =========================================================

  Logger::printfLog(
      Logger::INFO,
      "Final route cost: %d",
      finalRoute.cost);

  Logger::printfLog(
      Logger::INFO,
      "Final route size: %d",
      static_cast<int>(
          finalRoute.route.size()));

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
        "FinalRoute[%d]: "
        "grid=(%d,%d), "
        "position=(%.2f, %.2f), "
        "direction=%s",
        static_cast<int>(i),
        state.x,
        state.y,
        node.x,
        node.y,
        directionToString(
            state.direction));
  }

  // =========================================================
  // 13. 最終地点へ走行
  //
  // ここでもRouteFollower自身が、
  // ゲートを横切る区間だけCameraTrackingを使用する。
  // =========================================================

  routeFollower.run(
      finalRoute.route);

  // =========================================================
  // 14. 停止
  // =========================================================

  robot
      .getWheelMotorControllerInstance()
      .stopBoth();

  Logger::printfLog(
      Logger::INFO,
      "Final grid: (%d,%d), direction=%s",
      finalPoint.x,
      finalPoint.y,
      directionToString(
          finalDirection));

  Logger::info(
      "RouteFollower Square gate test finished");
}