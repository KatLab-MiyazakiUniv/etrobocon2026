/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

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
<<<<<<< HEAD
  Logger::info("GateRoutePlanner test start");

  // ==========================================
  // マップ情報を保持するクラス
  // ==========================================

  MapData mapData;

  /*
   * 本番では、ここをカメラやQRコードなどから
   * 取得したマップ情報に置き換える。
   *
   * 今回は動作確認のため直接登録する。
   */

  // 赤ゲート
  mapData.setGate(GoalColor::RED, { 1, 3 }, { 3, 3 });

  // 青ゲート
  mapData.setGate(GoalColor::BLUE, { 7, 3 }, { 7, 5 });

  // 黄ゲート
  mapData.setGate(GoalColor::YELLOW, { 3, 7 }, { 5, 7 });

  // ==========================================
  // 経路探索クラス
  // ==========================================

  GateRoutePlanner routePlanner(mapData);

  // ==========================================
  // 初期位置
  // ==========================================

  int currentX = 0;
  int currentY = 0;

  /*
   * 座標系
   *
   * UP    : Yが減る
   * RIGHT : Xが減る
   * DOWN  : Yが増える
   * LEFT  : Xが増える
   */
  Direction currentDirection = Direction::LEFT;

  // ==========================================
  // 通過するゲートの順番
  // ==========================================

  constexpr GoalColor TARGETS[] = { GoalColor::RED, GoalColor::BLUE, GoalColor::YELLOW };

  // ==========================================
  // 各ゲートについて経路探索
  // ==========================================

  for(const GoalColor targetColor : TARGETS) {
    Logger::info("--------------------------------");

    Logger::printfLog(Logger::INFO, "Current position: (%d, %d)", currentX, currentY);

    Logger::printfLog(Logger::INFO, "Current direction: %s", directionToString(currentDirection));

    Logger::printfLog(Logger::INFO, "Target gate: %s", colorToString(targetColor));

    // ========================================
    // 探索時間計測開始
    // ========================================

    auto startTime = std::chrono::high_resolution_clock::now();

    GateRouteResult result = routePlanner.search(currentX, currentY, currentDirection, targetColor);

    auto endTime = std::chrono::high_resolution_clock::now();

    // ========================================
    // 探索時間
    // ========================================

    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

    Logger::printfLog(Logger::INFO, "Search time: %lld us",
                      static_cast<long long>(elapsedTime.count()));

    // ========================================
    // 探索失敗
    // ========================================

    if(!result.found) {
      Logger::printfLog(Logger::ERROR, "Route to %s was not found", colorToString(targetColor));

      return;
    }

    // ========================================
    // 探索結果
    // ========================================

    Logger::printfLog(Logger::INFO, "Total cost: %d", result.cost);

    Logger::printfLog(Logger::INFO, "Gate entrance: (%d, %d)", result.entrance.x,
                      result.entrance.y);

    Logger::printfLog(Logger::INFO, "Gate exit: (%d, %d)", result.exit.x, result.exit.y);

    Logger::printfLog(Logger::INFO, "Exit direction: %s", directionToString(result.exitDirection));

    Logger::printfLog(Logger::INFO, "Route size: %d", static_cast<int>(result.route.size()));

    // ========================================
    // 圧縮後の経路を表示
    // ========================================

    for(size_t i = 0; i < result.route.size(); ++i) {
      const RouteState& state = result.route[i];

      Logger::printfLog(Logger::INFO, "Route[%d]: (%d, %d), direction=%s", static_cast<int>(i),
                        state.x, state.y, directionToString(state.direction));
    }

    // ========================================
    // ゲートを通過したものとして現在位置更新
    // ========================================

    currentX = result.exit.x;

    currentY = result.exit.y;

    currentDirection = result.exitDirection;

    Logger::printfLog(Logger::INFO, "Gate %s passed", colorToString(targetColor));
  }

  Logger::info("GateRoutePlanner test finished");
=======
  Logger::info("Hello KATLAB");
>>>>>>> ticket-KL26-142
}