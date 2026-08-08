/**
 * @file   EtRobocon2026.cpp
 * @brief  経路探索とGoalNavigationを使用したテスト走行
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

#include <memory>

#include "EtRallyMap.h"
#include "GateRoutePlanner.h"
#include "GoalNavigation.h"
#include "Logger.h"
#include "MapData.h"
#include "Pid.h"
#include "RealNetworkSystem.h"
#include "RepeatCountCondition.h"
#include "Robot.h"
#include "RouteTypes.h"
#include "SocketClient.h"

namespace {

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

}  // namespace

void EtRobocon2026::start()
{
  Logger::info("Map navigation test start");

  // =========================================================
  // 1. Robot生成
  // =========================================================

  RealNetworkSystem networkSystem;

  SocketClient cameraSocketClient(
      networkSystem);

  Robot robot(
      cameraSocketClient);

  // =========================================================
  // 2. マップ情報を保持するクラス
  // =========================================================

  MapData mapData;

  /*
   * 本番では走行中に取得したゲート情報を
   * MapData::setGate()で登録する。
   *
   * 今回はテストなので仮のゲート情報を登録する。
   */

  // 赤ゲート
  mapData.setGate(
      GoalColor::RED,
      { 5, 1 },
      { 7, 1 });

  // 青ゲート
  mapData.setGate(
      GoalColor::BLUE,
      { 1, 5 },
      { 1, 7 });

  // 黄ゲート
  mapData.setGate(
      GoalColor::YELLOW,
      { 7, 3 },
      { 9, 3 });

  // =========================================================
  // 3. 経路探索クラス・実座標変換クラス
  // =========================================================

  GateRoutePlanner routePlanner(
      mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 4. 現在位置
  // =========================================================

  /*
   * 格子座標
   *
   * UP    : Yが減る
   * RIGHT : Xが減る
   * DOWN  : Yが増える
   * LEFT  : Xが増える
   */

  int currentGridX = 0;
  int currentGridY = 0;

  Direction currentDirection
      = Direction::LEFT;

  // =========================================================
  // 5. 今回向かうゲート
  // =========================================================

  constexpr GoalColor TARGET_COLOR
      = GoalColor::RED;

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
    0.02,
    0.006,
    0.003
  };

  // テストなので低めの速度
  constexpr double TARGET_SPEED
      = 200.0;

  // =========================================================
  // 7. オドメトリ初期化
  // =========================================================

  WheelMotorController& wheelMotorController
      = robot.getWheelMotorControllerInstance();

  Odometry& odometry
      = robot.getOdometryInstance();

  odometry.initialize(
      wheelMotorController.getLeftCount(),
      wheelMotorController.getRightCount());

  // =========================================================
  // 8. ゲートまで経路探索
  // =========================================================

  GateRouteResult routeResult
      = routePlanner.search(
          currentGridX,
          currentGridY,
          currentDirection,
          TARGET_COLOR);

  if(!routeResult.found) {
    Logger::error(
        "Route not found");

    return;
  }

  // =========================================================
  // 9. 探索結果表示
  // =========================================================

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
      directionToString(
          routeResult.exitDirection));

  Logger::printfLog(
      Logger::INFO,
      "Compressed route size: %d",
      static_cast<int>(
          routeResult.route.size()));

  // =========================================================
  // 10. 圧縮された経路を順番に走行
  // =========================================================

  /*
   * route[0]は現在地点なので、
   * route[1]からGoalNavigationで走る。
   */

  for(size_t i = 1;
      i < routeResult.route.size();
      ++i) {

    const RouteState& targetState
        = routeResult.route[i];

    // -------------------------------------------------------
    // 格子座標 → 実座標[mm]
    // -------------------------------------------------------

    EtRallyMap::Node targetNode
        = etRallyMap.getNode(
            targetState.x,
            targetState.y);

    double targetX
        = targetNode.x;

    double targetY
        = targetNode.y;

    Logger::printfLog(
        Logger::INFO,
        "Target[%d]: grid=(%d,%d), position=(%.2f, %.2f), direction=%s",
        static_cast<int>(i),
        targetState.x,
        targetState.y,
        targetX,
        targetY,
        directionToString(
            targetState.direction));

    // -------------------------------------------------------
    // GoalNavigation実行
    // -------------------------------------------------------
Position& position = robot.getPositionInstance(); 
Logger::printfLog( Logger::INFO, "Current real position: (%.2f, %.2f), heading=%.2f", position.getX(), position.getY(), position.getHeading());





    auto condition
        = std::make_unique<
            RepeatCountCondition>(
                robot,
                1);

    GoalNavigation goalNavigation(
        robot,
        std::move(condition),
        targetX,
        targetY,
        TARGET_SPEED,
        rotationPid,
        rightPid,
        leftPid,
        straightAnglePid);

    goalNavigation.run();

    // -------------------------------------------------------
    // 現在の格子位置を更新
    // -------------------------------------------------------

    currentGridX
        = targetState.x;

    currentGridY
        = targetState.y;

    currentDirection
        = targetState.direction;

    Logger::printfLog(
        Logger::INFO,
        "Reached: grid=(%d,%d), direction=%s",
        currentGridX,
        currentGridY,
        directionToString(
            currentDirection));
  }

  // =========================================================
  // 11. 終了
  // =========================================================

  Logger::printfLog(
      Logger::INFO,
      "Gate passed: exit=(%d,%d), direction=%s",
      routeResult.exit.x,
      routeResult.exit.y,
      directionToString(
          routeResult.exitDirection));

  Logger::info(
      "Map navigation test finished");
}
