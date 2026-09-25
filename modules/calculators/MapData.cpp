/**
 * @file   MapData.cpp
 * @brief  走行中に取得したマップ情報を保持するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "MapData.h"

namespace {

  /**
   * @brief 受信したX座標をマップ上のX座標へ変換する
   * @param x 受信したX座標
   * @return 変換後のX座標
   */
  int convertX(int x)
  {
    switch(x) {
      case 1:
        return 9;

      case 2:
        return 7;

      case 3:
        return 5;

      case 4:
        return 3;

      case 5:
        return 1;
    }

    return -1;
  }

  /**
   * @brief 受信したY座標をマップ上のY座標へ変換する
   * @param y 受信したY座標
   * @return 変換後のY座標
   */
  int convertY(int y)
  {
    switch(y) {
      case 1:
        return 1;

      case 2:
        return 3;

      case 3:
        return 5;

      case 4:
        return 7;

      case 5:
        return 9;
    }

    return -1;
  }

  /**
   * @brief 2桁の座標番号をマップ上の座標へ変換する
   * @param pointNumber 2桁の座標番号
   * @param point 変換後の座標
   * @return true/変換成功、false/変換失敗
   */
  bool convertPointNumber(int pointNumber, Point& point)
  {
    // 10の位をX座標、1の位をY座標として取得
    const int x = pointNumber / 10;
    const int y = pointNumber % 10;

    // 受信した座標をマップ上の座標へ変換
    const int convertedX = convertX(x);
    const int convertedY = convertY(y);

    // 変換できない座標の場合
    if(convertedX < 0 || convertedY < 0) {
      return false;
    }

    point = { convertedX, convertedY };

    return true;
  }

}  // namespace

MapData::MapData() : gates() {}

void MapData::setGate(GoalColor color, const Point& start, const Point& end)
{
  // 同じ色のゲートがすでに存在する場合は更新
  for(Gate& gate : gates) {
    if(gate.color == color) {
      gate.start = start;
      gate.end = end;

      return;
    }
  }

  // 存在しない場合は新しく追加
  gates.push_back({ color, start, end });
}

bool MapData::setGate(GoalColor color, int startNumber, int endNumber)
{
  Point start;
  Point end;

  // 始点の座標番号をマップ上の座標へ変換
  if(!convertPointNumber(startNumber, start)) {
    return false;
  }

  // 終点の座標番号をマップ上の座標へ変換
  if(!convertPointNumber(endNumber, end)) {
    return false;
  }

  // 変換した座標を使用してゲート情報を登録
  setGate(color, start, end);

  return true;
}

bool MapData::hasGate(GoalColor color) const
{
  return getGate(color) != nullptr;
}

const Gate* MapData::getGate(GoalColor color) const
{
  for(const Gate& gate : gates) {
    if(gate.color == color) {
      return &gate;
    }
  }

  return nullptr;
}

const std::vector<Gate>& MapData::getGates() const
{
  return gates;
}

std::vector<GatePass> MapData::getGatePasses(GoalColor color) const
{
  std::vector<GatePass> passes;

  const Gate* gate = getGate(color);

  // 指定色のゲート情報がまだない
  if(gate == nullptr) {
    return passes;
  }

  // 横向きゲート
  if(gate->start.y == gate->end.y) {
    int centerX = (gate->start.x + gate->end.x) / 2;
    int gateY = gate->start.y;

    // upからdown
    passes.push_back({ { centerX, gateY - 1 }, { centerX, gateY + 1 }, Direction::DOWN });
    // downからup
    passes.push_back({ { centerX, gateY + 1 }, { centerX, gateY - 1 }, Direction::UP });

    return passes;
  }

  // 縦向きゲート
  if(gate->start.x == gate->end.x) {
    int centerY = (gate->start.y + gate->end.y) / 2;
    int gateX = gate->start.x;

    // leftからright
    passes.push_back({ { gateX + 1, centerY }, { gateX - 1, centerY }, Direction::RIGHT });
    // rightからleft
    passes.push_back({ { gateX - 1, centerY }, { gateX + 1, centerY }, Direction::LEFT });
  }

  return passes;
}