/**
 * @file   MapData.cpp
 * @brief  走行中に取得したマップ情報を保持するクラス
 * @author okuyama0528 yutaro-1214
 */

#include "MapData.h"

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

    passes.push_back({ { centerX, gateY - 1 }, { centerX, gateY + 1 }, Direction::DOWN });
    passes.push_back({ { centerX, gateY + 1 }, { centerX, gateY - 1 }, Direction::UP });

    return passes;
  }

  // 縦向きゲート
  if(gate->start.x == gate->end.x) {
    int centerY = (gate->start.y + gate->end.y) / 2;
    int gateX = gate->start.x;

    passes.push_back({ { gateX + 1, centerY }, { gateX - 1, centerY }, Direction::RIGHT });
    passes.push_back({ { gateX - 1, centerY }, { gateX + 1, centerY }, Direction::LEFT });
  }

  return passes;
}