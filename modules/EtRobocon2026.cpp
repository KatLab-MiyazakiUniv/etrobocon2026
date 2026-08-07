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
#include <vector>

#include "DijkstraRoutePlanner.h"

namespace {

  /**
   * @brief Directionを文字列へ変換する
   */
  const char* directionToString(Direction direction)
  {
    switch(direction) {
      case Direction::NORTH:
        return "NORTH";

      case Direction::EAST:
        return "EAST";

      case Direction::SOUTH:
        return "SOUTH";

      case Direction::WEST:
        return "WEST";
    }

    return "UNKNOWN";
  }

}  // namespace

void EtRobocon2026::start()
{
  Logger::info("DijkstraRoutePlanner test start");

  // =========================
  // ゲート
  // =========================

  std::vector<Gate> gates = {

    // (2,2) ↔ (2,4) を遮断
    { { 1, 5 }, { 1, 7 } },

    // (6,4) ↔ (8,4) を遮断
    { { 5, 9 }, { 7, 9 } },

    // (4,6) ↔ (4,8) を遮断
    { { 7, 3 }, { 9, 3 } }
  };

  // =========================
  // スタート地点
  // =========================

  constexpr int START_X = 0;
  constexpr int START_Y = 0;

  constexpr Direction START_DIRECTION = Direction::WEST;

  // =========================
  // ゴール候補
  // =========================

  const Point goal1 = { 6, 8 };

  const Point goal2 = { 6, 10 };

  // =========================
  // 経路探索クラス生成
  // =========================

  DijkstraRoutePlanner planner(gates);

  // =========================
  // 時間計測開始
  // =========================

  auto startTime = std::chrono::high_resolution_clock::now();

  RouteResult result = planner.search(START_X, START_Y, START_DIRECTION, goal1, goal2);

  // =========================
  // 時間計測終了
  // =========================

  auto endTime = std::chrono::high_resolution_clock::now();

  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

  Logger::printfLog(Logger::INFO, "Search time: %lld us",
                    static_cast<long long>(elapsedTime.count()));

  // =========================
  // 経路が見つからない
  // =========================

  if(!result.found) {
    Logger::info("Route not found");

    return;
  }

  // =========================
  // 探索結果
  // =========================

  Logger::printfLog(Logger::INFO, "Selected goal: (%d, %d)", result.goal.x, result.goal.y);

  Logger::printfLog(Logger::INFO, "Total cost: %d", result.cost);

  Logger::printfLog(Logger::INFO, "Route size: %d", static_cast<int>(result.route.size()));

  // =========================
  // 経路表示
  // =========================

  for(const RouteState& state : result.route) {
    Logger::printfLog(Logger::INFO, "(%d, %d) direction=%s", state.x, state.y,
                      directionToString(state.direction));
  }

  Logger::info("DijkstraRoutePlanner test end");
}
