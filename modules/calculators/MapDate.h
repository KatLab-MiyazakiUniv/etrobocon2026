/**
 * @file   MapData.h
 * @brief  固定マップ情報を保持するクラス
 */

#ifndef MAP_DATA_H
#define MAP_DATA_H

#include <vector>

#include "RouteTypes.h"

/**
 * @brief マップ情報を管理するクラス
 */
class MapData {
 public:
  MapData();

  /**
   * @brief 全ゲートを取得する
   */
  const std::vector<Gate>& getGates() const;

  /**
   * @brief 指定色のゲートを取得する
   */
  const Gate& getGate(GoalColor color) const;

  /**
   * @brief 指定色のゲートの通過候補を取得する
   *
   * 返される2つは、
   * 両方向からゲートをくぐる場合の情報。
   */
  std::vector<GatePass> getGatePasses(GoalColor color) const;

 private:
  std::vector<Gate> gates;
};

#endif  // MAP_DATA_H