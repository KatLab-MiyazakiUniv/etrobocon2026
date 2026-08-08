/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "AreaMaster.h"
#include "Robot.h"
#include "Course.h"
#include "CsvLogger.h"
#include "RealNetworkSystem.h"
#include "BatteryController.h"
#include "Robot.h"
#include "DistanceCondition.h"
#include "Pid.h"
#include "CameraTracking.h"
#include <vector>

#include <chrono>

#include "GateRoutePlanner.h"
#include "MapData.h"
#include "RouteTypes.h"

namespace {

  /**
   * @brief Directionを文字列に変換する
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
   * @brief GoalColorを文字列に変換する
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
  Logger::info("GateRoutePlanner sequential test start");

  MapData mapData;

  GateRoutePlanner routePlanner(mapData);

  int currentX = 0;
  int currentY = 0;

  Direction currentDirection = Direction::LEFT;

  constexpr GoalColor TARGETS[] = { GoalColor::RED, GoalColor::BLUE, GoalColor::YELLOW };

  for(const GoalColor targetColor : TARGETS) {
    Logger::printfLog(Logger::INFO, "--------------------------------");

    Logger::printfLog(Logger::INFO, "Current: (%d, %d), direction=%s", currentX, currentY,
                      directionToString(currentDirection));

    Logger::printfLog(Logger::INFO, "Next gate: %s", colorToString(targetColor));

    auto startTime = std::chrono::high_resolution_clock::now();

    GateRouteResult result = routePlanner.search(currentX, currentY, currentDirection, targetColor);

    auto endTime = std::chrono::high_resolution_clock::now();

    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

    Logger::printfLog(Logger::INFO, "Search time: %lld us",
                      static_cast<long long>(elapsedTime.count()));

    if(!result.found) {
      Logger::printfLog(Logger::ERROR, "Route to %s not found", colorToString(targetColor));

      return;
    }

    Logger::printfLog(Logger::INFO, "Cost: %d", result.cost);

    Logger::printfLog(Logger::INFO, "Entrance: (%d, %d)", result.entrance.x, result.entrance.y);

    Logger::printfLog(Logger::INFO, "Exit: (%d, %d)", result.exit.x, result.exit.y);

    for(size_t i = 0; i < result.route.size(); ++i) {
      const RouteState& state = result.route[i];

      Logger::printfLog(Logger::INFO, "Route[%d]: (%d, %d), direction=%s", static_cast<int>(i),
                        state.x, state.y, directionToString(state.direction));
    }

    /*
     * ゲートをくぐり切ったものとして、
     * 次回探索の開始状態を更新する。
     */
    currentX = result.exit.x;

    currentY = result.exit.y;

    currentDirection = result.exitDirection;
  }

  Logger::info("GateRoutePlanner sequential test end");
}
