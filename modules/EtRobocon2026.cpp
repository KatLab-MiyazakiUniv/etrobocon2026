/**
 * @file   EtRobocon2026.cpp
 * @brief  (8,8)から赤ゲート(7,9)-(9,9)を通過するテスト
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

}  // namespace

void EtRobocon2026::start()
{
  Logger::info("RED gate RouteFollower test start");

  // =========================================================
  // 1. Robot生成
  // =========================================================

  RealNetworkSystem networkSystem;

  SocketClient cameraSocketClient(networkSystem);

  Robot robot(cameraSocketClient);

  // =========================================================
  // 2. ゲート情報
  // =========================================================

  MapData mapData;

  /*
   * 赤ゲート
   *
   * (7,9) -------- (9,9)
   *
   * 現在位置は(8,8)なので、
   * DOWN方向に進んでゲートを通過する想定。
   */
  mapData.setGate(GoalColor::RED, { 7, 9 }, { 9, 9 });

  // =========================================================
  // 3. 経路探索
  // =========================================================

  GateRoutePlanner routePlanner(mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 4. 開始状態
  // =========================================================

  constexpr int START_GRID_X = 8;
  constexpr int START_GRID_Y = 8;

  /*
   * 最初からゲート方向を向いている状態でテストする。
   *
   * DOWN : Yが増える
   */
  Direction currentDirection = Direction::DOWN;

  Logger::printfLog(Logger::INFO, "Start grid: (%d,%d), direction=%s", START_GRID_X, START_GRID_Y,
                    directionToString(currentDirection));

  // =========================================================
  // 5. PID設定
  // =========================================================

  const Pid::PidGain rotationPid = { 1.3, 1.0, 0.0 };

  const Pid::PidGain rightPid = { 0.016, 0.005, 0.0015 };

  const Pid::PidGain leftPid = { 0.016, 0.0045, 0.0015 };

  const Pid::PidGain straightAnglePid = { 0.033, 0.003, 0.03 };

  // テストなので低速
  constexpr double TARGET_SPEED = 300.0;

  // =========================================================
  // 6. RouteFollower生成
  // =========================================================

  RouteFollower routeFollower(robot, etRallyMap, TARGET_SPEED, rotationPid, rightPid, leftPid,
                              straightAnglePid);

  // =========================================================
  // 7. オドメトリ初期化
  // =========================================================

  WheelMotorController& wheelMotorController = robot.getWheelMotorControllerInstance();

  Odometry& odometry = robot.getOdometryInstance();

  odometry.initialize(wheelMotorController.getLeftCount(), wheelMotorController.getRightCount());

  // =========================================================
  // 8. 赤ゲートへの経路探索
  // =========================================================

  GateRouteResult routeResult
      = routePlanner.search(START_GRID_X, START_GRID_Y, currentDirection, GoalColor::RED);

  if(!routeResult.found) {
    Logger::error("Route to RED gate not found");

    robot.getWheelMotorControllerInstance().stopBoth();

    return;
  }

  // =========================================================
  // 9. 探索結果表示
  // =========================================================

  Logger::printfLog(Logger::INFO, "Route cost: %d", routeResult.cost);

  Logger::printfLog(Logger::INFO, "Gate entrance: (%d,%d)", routeResult.entrance.x,
                    routeResult.entrance.y);

  Logger::printfLog(Logger::INFO, "Gate exit: (%d,%d)", routeResult.exit.x, routeResult.exit.y);

  Logger::printfLog(Logger::INFO, "Exit direction: %s",
                    directionToString(routeResult.exitDirection));

  Logger::printfLog(Logger::INFO, "Compressed route size: %d",
                    static_cast<int>(routeResult.route.size()));

  // =========================================================
  // 10. 経路と実座標を表示
  // =========================================================

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

  // =========================================================
  // 11. 距離確認
  // =========================================================

  const EtRallyMap::Node startNode = etRallyMap.getNode(8, 8);

  const EtRallyMap::Node gateEndNode = etRallyMap.getNode(8, 10);

  Logger::printfLog(Logger::INFO, "Expected Y distance: %.2f mm",
                    std::abs(gateEndNode.y - startNode.y));

  // =========================================================
  // 12. 実際に走行
  // =========================================================

  routeFollower.run(routeResult.route);

  // =========================================================
  // 13. 終了
  // =========================================================

  robot.getWheelMotorControllerInstance().stopBoth();

  Logger::printfLog(Logger::INFO, "RED gate passed: exit=(%d,%d), direction=%s", routeResult.exit.x,
                    routeResult.exit.y, directionToString(routeResult.exitDirection));

  Logger::info("RED gate RouteFollower test finished");
}