/**
 * @file   MapData.h
 * @brief  走行中に取得したマップ情報を保持するクラス
 * @author okuyama0528 yutaro-1214
 */

#ifndef MAP_DATA_H
#define MAP_DATA_H

#include <vector>
#include "RouteTypes.h"

class MapData {
 public:
  /**
   * @brief コンストラクタ
   */
  MapData();

  /**
   * @brief ゲート情報を登録・更新する
   * @param color ゲートの色
   * @param start ゲートの始点
   * @param end ゲートの終点
   */
  void setGate(GoalColor color, const Point& start, const Point& end);

  /**
   * @brief 指定した色のゲート情報が存在するか確認する
   * @param color ゲートの色
   * @return true/ゲート情報が存在する、false/ゲート情報が存在しない
   */
  bool hasGate(GoalColor color) const;

  /**
   * @brief 指定した色のゲートを取得する
   * @param color ゲートの色
   * @return ゲートへのポインタ
   */
  const Gate* getGate(GoalColor color) const;

  /**
   * @brief 現在登録されている全ゲートを取得する
   */
  const std::vector<Gate>& getGates() const;

  /**
   * @brief 指定したゲートを通過する2つの候補を取得する
   * @param color ゲートの色
   * @return ゲート通過候補
   */
  std::vector<GatePass> getGatePasses(GoalColor color) const;

 private:
  std::vector<Gate> gates;  // ゲートの情報を保持
};

#endif  // MAP_DATA_H