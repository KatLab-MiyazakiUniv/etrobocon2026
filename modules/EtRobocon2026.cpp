```cpp
/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

#include <memory>

#include "EtRallyMap.h"
#include "GateRoutePlanner.h"
#include "MapData.h"
#include "RepeatCountCondition.h"
#include "RouteTypes.h"

    namespace
{
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
  // 1. MAP情報を用意
  // =========================================================

  MapData mapData;

  /*
   * 本番では、走行中に取得したMAP情報を
   * setGate()で登録する。
   *
   * 今回は実走テストなので仮の値を直接登録する。
   */

  // 赤ゲート
  mapData.setGate(GoalColor::RED, { 1, 3 }, { 3, 3 });

  // 青ゲート
  mapData.setGate(GoalColor::BLUE, { 7, 3 }, { 7, 5 });

  // 黄ゲート
  mapData.setGate(GoalColor::YELLOW, { 3, 7 }, { 5, 7 });

  // =========================================================
  // 2. 経路探索と実座標変換用クラス
  // =========================================================

  GateRoutePlanner routePlanner(mapData);

  EtRallyMap etRallyMap;

  // =========================================================
  // 3. 現在の格子座標・向き
  // =========================================================

  int currentGridX = 0;
  int currentGridY = 0;

  /*
   * UP    : Yが減る
   * RIGHT : Xが減る
   * DOWN  : Yが増える
   * LEFT  : Xが増える
   */
  Direction currentDirection = Direction::LEFT;

  // =========================================================
  // 4. 今回向かうゲート
  // =========================================================

  constexpr GoalColor TARGET_COLOR = GoalColor::RED;

  // =========================================================
  // 5. PID設定
  // =========================================================

  const Pid::PidGain rotationPid = { 1.3, 1.0, 0.0 };

  const Pid::PidGain rightPid = { 0.016, 0.005, 0.0015 };

  const Pid::PidGain leftPid = { 0.016, 0.0045, 0.0015 };

  const Pid::PidGain straightAnglePid = { 0.02, 0.006, 0.003 };

  // 直進速度[mm/s]
  constexpr double TARGET_SPEED = 300.0;

  // =========================================================
  // 6. オドメトリ初期化
  // =========================================================

  WheelMotorController& wheelMotorController = robot.getWheelMotorControllerInstance();

  Odometry& odometry = robot.getOdometryInstance();

  odometry.initialize(wheelMotorController.getLeftCount(), wheelMotorController.getRightCount());

  // =========================================================
  // 7. 経路探索
  // =========================================================

  GateRouteResult routeResult
      = routePlanner.search(currentGridX, currentGridY, currentDirection, TARGET_COLOR);

  if(!routeResult.found) {
    Logger::error("Route not found");
    return;
  }

  Logger::printfLog(Logger::INFO, "Gate entrance: (%d, %d)", routeResult.entrance.x,
                    routeResult.entrance.y);

  Logger::printfLog(Logger::INFO, "Gate exit: (%d, %d)", routeResult.exit.x, routeResult.exit.y);

  Logger::printfLog(Logger::INFO, "Route cost: %d", routeResult.cost);

  Logger::printfLog(Logger::INFO, "Route size: %d", static_cast<int>(routeResult.route.size()));

  // =========================================================
  // 8. 圧縮された経路を順番に走る
  // =========================================================

  /*
   * route[0]は現在位置なので、
   * route[1]から走行する。
   */

  for(size_t i = 1; i < routeResult.route.size(); ++i) {
    const RouteState& targetState = routeResult.route[i];

    // ===============================================
    // 格子座標 → 実座標[mm]
    // ===============================================

    EtRallyMap::Node targetNode = etRallyMap.getNode(targetState.x, targetState.y);

    double targetX = targetNode.x;

    double targetY = targetNode.y;

    Logger::printfLog(Logger::INFO, "Target[%d]: grid=(%d,%d), position=(%.2f,%.2f), direction=%s",
                      static_cast<int>(i), targetState.x, targetState.y, targetX, targetY,
                      directionToString(targetState.direction));

    // ===============================================
    // GoalNavigationを1回だけ実行
    // ===============================================

    auto condition = std::make_unique<RepeatCountCondition>(robot, 1);

    GoalNavigation goalNavigation(robot, std::move(condition), targetX, targetY, TARGET_SPEED,
                                  rotationPid, rightPid, leftPid, straightAnglePid);

    goalNavigation.run();

    /*
     * この地点まで実際に走行できたので、
     * 現在の格子座標と向きを更新する。
     */
    currentGridX = targetState.x;

    currentGridY = targetState.y;

    currentDirection = targetState.direction;

    Logger::printfLog(Logger::INFO, "Reached grid=(%d,%d)", currentGridX, currentGridY);
  }

  // =========================================================
  // 9. ゲート通過後の確認
  // =========================================================

  Logger::printfLog(Logger::INFO, "Gate passed: exit=(%d,%d), direction=%s", routeResult.exit.x,
                    routeResult.exit.y, directionToString(routeResult.exitDirection));

  Logger::info("Map navigation test finished");
}
```
