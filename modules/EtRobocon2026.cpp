/**
 * @file   EtRobocon2026.cpp
 * @brief  3色のゲートを順番に経路探索してRouteFollowerで走行するテスト
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

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
  Logger::info("Three gate RouteFollower test start");

  // =========================================================
  // 1. Robot生成
  // =========================================================

  RealNetworkSystem networkSystem;

  SocketClient cameraSocketClient(networkSystem);

  Robot robot(cameraSocketClient);

  // =========================================================
  // 2. ゲート情報登録
  // =========================================================

  MapData mapData;

  // 赤ゲート
  // (5,9) ----- (7,9)
  mapData.setGate(GoalColor::RED, { 7, 9 }, { 9, 9 });

  // 青ゲート
  // (1,5)
  //   |
  // (1,7)
  mapData.setGate(GoalColor::BLUE, { 3, 5 }, { 3, 7 });

  // 黄ゲート
  // (7,3) ----- (9,3)
  mapData.setGate(GoalColor::YELLOW, { 7, 3 }, { 9, 3 });

  // =========================================================
  // 3. 経路探索・距離変換
  // =========================================================

  GateRoutePlanner routePlanner(mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 4. 開始位置
  // =========================================================

  int currentGridX = 2;
  int currentGridY = 2;

  /*
   * 格子座標系
   *
   * UP    : Yが減る
   * RIGHT : Xが減る
   * DOWN  : Yが増える
   * LEFT  : Xが増える
   */
  Direction currentDirection = Direction::LEFT;

  // =========================================================
  // 5. 通過するゲートの順番
  // =========================================================

  constexpr GoalColor TARGET_COLORS[] = { GoalColor::RED, GoalColor::BLUE, GoalColor::YELLOW };

  // =========================================================
  // 6. PID設定
  // =========================================================

  const Pid::PidGain rotationPid = { 1.3, 1.0, 0.0 };

  const Pid::PidGain rightPid = { 0.016, 0.005, 0.0015 };

  const Pid::PidGain leftPid = { 0.016, 0.0045, 0.0015 };

  const Pid::PidGain straightAnglePid = { 0.033, 0.003, 0.03 };

  constexpr double TARGET_SPEED = 300.0;

  // =========================================================
  // 7. RouteFollower生成
  // =========================================================

  RouteFollower routeFollower(robot, etRallyMap, TARGET_SPEED, rotationPid, rightPid, leftPid,
                              straightAnglePid);

  // =========================================================
  // 8. オドメトリ初期化
  // =========================================================

  WheelMotorController& wheelMotorController = robot.getWheelMotorControllerInstance();

  Odometry& odometry = robot.getOdometryInstance();

  odometry.initialize(wheelMotorController.getLeftCount(), wheelMotorController.getRightCount());

  // =========================================================
  // 9. RED → BLUE → YELLOW を3周走行
  // =========================================================

  constexpr int LAP_COUNT = 3;

  for(int lap = 1; lap <= LAP_COUNT; ++lap) {
    Logger::info("========================================");

    Logger::printfLog(Logger::INFO, "Lap %d / %d start", lap, LAP_COUNT);

    // RED → BLUE → YELLOW
    for(const GoalColor targetColor : TARGET_COLORS) {
      Logger::info("==============================");

      Logger::printfLog(Logger::INFO, "Lap %d / %d, Target gate: %s", lap, LAP_COUNT,
                        colorToString(targetColor));

      Logger::printfLog(Logger::INFO, "Current grid: (%d, %d), direction=%s", currentGridX,
                        currentGridY, directionToString(currentDirection));

      // -------------------------------------------------------
      // 現在位置から指定された色のゲートまで経路探索
      // -------------------------------------------------------

      GateRouteResult routeResult
          = routePlanner.search(currentGridX, currentGridY, currentDirection, targetColor);

      if(!routeResult.found) {
        Logger::printfLog(Logger::ERROR, "Route to %s not found", colorToString(targetColor));

        robot.getWheelMotorControllerInstance().stopBoth();

        return;
      }

      // -------------------------------------------------------
      // 経路探索結果を表示
      // -------------------------------------------------------

      Logger::printfLog(Logger::INFO, "Route cost: %d", routeResult.cost);

      Logger::printfLog(Logger::INFO, "Gate entrance: (%d, %d)", routeResult.entrance.x,
                        routeResult.entrance.y);

      Logger::printfLog(Logger::INFO, "Gate exit: (%d, %d)", routeResult.exit.x,
                        routeResult.exit.y);

      Logger::printfLog(Logger::INFO, "Exit direction: %s",
                        directionToString(routeResult.exitDirection));

      Logger::printfLog(Logger::INFO, "Compressed route size: %d",
                        static_cast<int>(routeResult.route.size()));

      // -------------------------------------------------------
      // 経路の内容を表示
      // -------------------------------------------------------

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

      // -------------------------------------------------------
      // RouteFollowerで走行
      // -------------------------------------------------------

      routeFollower.run(routeResult.route);

      // -------------------------------------------------------
      // 次の探索開始位置を更新
      // -------------------------------------------------------

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
  // 10. 3周完了
  // =========================================================

  robot.getWheelMotorControllerInstance().stopBoth();

  Logger::printfLog(Logger::INFO, "Final grid: (%d, %d), direction=%s", currentGridX, currentGridY,
                    directionToString(currentDirection));

  Logger::info("Three laps RouteFollower test finished");
}