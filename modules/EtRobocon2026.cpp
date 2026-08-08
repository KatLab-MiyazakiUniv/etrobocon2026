```cpp
/**
 * @file   EtRobocon2026.cpp
 * @brief  3色のゲートを順番に経路探索して走行するテスト
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

    namespace
{
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
  Logger::info("Three gate navigation test start");

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
  mapData.setGate(GoalColor::RED, { 5, 9 }, { 7, 9 });

  // 青ゲート
  // (1,5)
  //   |
  // (1,7)
  mapData.setGate(GoalColor::BLUE, { 1, 5 }, { 1, 7 });

  // 黄ゲート
  // (7,3) ----- (9,3)
  mapData.setGate(GoalColor::YELLOW, { 7, 3 }, { 9, 3 });

  // =========================================================
  // 3. 経路探索・座標変換
  // =========================================================

  GateRoutePlanner routePlanner(mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 4. 開始位置
  // =========================================================

  int currentGridX = 0;
  int currentGridY = 0;

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

  const Pid::PidGain straightAnglePid = { 0.02, 0.006, 0.003 };

  // テスト走行なので低めに設定
  constexpr double TARGET_SPEED = 200.0;

  // =========================================================
  // 7. オドメトリ初期化
  // =========================================================

  WheelMotorController& wheelMotorController = robot.getWheelMotorControllerInstance();

  Odometry& odometry = robot.getOdometryInstance();

  odometry.initialize(wheelMotorController.getLeftCount(), wheelMotorController.getRightCount());

  // =========================================================
  // 8. RED → BLUE → YELLOW の順番で走行
  // =========================================================

  for(const GoalColor targetColor : TARGET_COLORS) {
    Logger::info("==============================");

    Logger::printfLog(Logger::INFO, "Target gate: %s", colorToString(targetColor));

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

    Logger::printfLog(Logger::INFO, "Gate exit: (%d, %d)", routeResult.exit.x, routeResult.exit.y);

    Logger::printfLog(Logger::INFO, "Exit direction: %s",
                      directionToString(routeResult.exitDirection));

    Logger::printfLog(Logger::INFO, "Compressed route size: %d",
                      static_cast<int>(routeResult.route.size()));

    // -------------------------------------------------------
    // 圧縮された経路を順番にGoalNavigationで走行
    // -------------------------------------------------------

    /*
     * route[0]は現在位置。
     * そのためroute[1]から走行する。
     */
    for(size_t i = 1; i < routeResult.route.size(); ++i) {
      const RouteState& targetState = routeResult.route[i];

      // 格子座標を実座標[mm]へ変換
      EtRallyMap::Node targetNode = etRallyMap.getNode(targetState.x, targetState.y);

      const double targetX = targetNode.x;
      const double targetY = targetNode.y;

      Position& position = robot.getPositionInstance();

      Logger::printfLog(Logger::INFO,
                        "Target[%d]: grid=(%d,%d), "
                        "position=(%.2f, %.2f), direction=%s",
                        static_cast<int>(i), targetState.x, targetState.y, targetX, targetY,
                        directionToString(targetState.direction));

      Logger::printfLog(Logger::INFO,
                        "Current real position: "
                        "(%.2f, %.2f), heading=%.2f",
                        position.getX(), position.getY(), position.getHeading());

      // -----------------------------------------------------
      // GoalNavigationで目標座標まで移動
      // -----------------------------------------------------

      auto condition = std::make_unique<RepeatCountCondition>(robot, 1);

      GoalNavigation goalNavigation(robot, std::move(condition), targetX, targetY, TARGET_SPEED,
                                    rotationPid, rightPid, leftPid, straightAnglePid);

      goalNavigation.run();

      // -----------------------------------------------------
      // 格子上の現在位置と方向を更新
      // -----------------------------------------------------

      currentGridX = targetState.x;
      currentGridY = targetState.y;
      currentDirection = targetState.direction;

      Logger::printfLog(Logger::INFO, "Reached: grid=(%d,%d), direction=%s", currentGridX,
                        currentGridY, directionToString(currentDirection));
    }

    // =======================================================
    // ゲート通過後の状態を次の探索開始状態にする
    // =======================================================

    currentGridX = routeResult.exit.x;
    currentGridY = routeResult.exit.y;
    currentDirection = routeResult.exitDirection;

    Logger::printfLog(Logger::INFO, "%s gate passed: exit=(%d,%d), direction=%s",
                      colorToString(targetColor), currentGridX, currentGridY,
                      directionToString(currentDirection));
  }

  // =========================================================
  // 9. 全ゲート通過完了
  // =========================================================

  robot.getWheelMotorControllerInstance().stopBoth();

  Logger::printfLog(Logger::INFO, "Final grid: (%d, %d), direction=%s", currentGridX, currentGridY,
                    directionToString(currentDirection));

  Logger::info("Three gate navigation test finished");
}
```
