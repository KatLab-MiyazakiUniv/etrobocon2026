/**
 * @file   MapData.h
 * @brief  走行中に取得したマップ情報を保持するクラス
 */

#ifndef MAP_DATA_H
#define MAP_DATA_H

#include <vector>

#include "RouteTypes.h"

/**
 * @brief 走行中に取得したゲート情報を管理するクラス
 */
class MapData {
 public:
  MapData();

  /**
   * @brief ゲート情報を登録・更新する
   *
   * 同じ色のゲートがすでに存在する場合は更新する。
   *
   * @param color ゲートの色
   * @param start ゲートの始点
   * @param end ゲートの終点
   */
  void setGate(GoalColor color, const Point& start, const Point& end);

  /**
   * @brief 指定した色のゲート情報が存在するか確認する
   *
   * @param color ゲートの色
   * @return 存在する場合true
   */
  bool hasGate(GoalColor color) const;

  /**
   * @brief 指定した色のゲートを取得する
   *
   * @param color ゲートの色
   * @return ゲートへのポインタ
   *
   * 存在しない場合はnullptrを返す。
   */
  const Gate* getGate(GoalColor color) const;

  /**
   * @brief 現在登録されている全ゲートを取得する
   */
  const std::vector<Gate>& getGates() const;

  /**
   * @brief 指定したゲートを通過する2つの候補を取得する
   *
   * @param color ゲートの色
   * @return ゲート通過候補
   */
  std::vector<GatePass> getGatePasses(GoalColor color) const;

 private:
  std::vector<Gate> gates;
};

#endif  // MAP_DATA_H