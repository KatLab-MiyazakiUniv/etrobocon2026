/**
 * @file   MapData.cpp
 * @brief  固定マップ情報を保持するクラス
 */

#include "MapData.h"

MapData::MapData()
  : gates{ // 赤ゲート
           { GoalColor::RED, { 1, 3 }, { 3, 3 } },

           // 青ゲート
           { GoalColor::BLUE, { 7, 3 }, { 7, 5 } },

           // 黄ゲート
           { GoalColor::YELLOW, { 3, 7 }, { 5, 7 } }
    }
{
}

const std::vector<Gate>& MapData::getGates() const
{
  return gates;
}

const Gate& MapData::getGate(GoalColor color) const
{
  for(const Gate& gate : gates) {
    if(gate.color == color) {
      return gate;
    }
  }

  // RED / BLUE / YELLOWの3種類しか来ない前提
  return gates[0];
}

std::vector<GatePass> MapData::getGatePasses(GoalColor color) const
{
  const Gate& gate = getGate(color);

  std::vector<GatePass> passes;

  // ==========================================
  // 横向きゲート
  //
  // 例:
  //
  // (1,3) -------- (3,3)
  //
  //        (2,2)
  //          ↓
  //        (2,4)
  // ==========================================

  if(gate.start.y == gate.end.y) {
    int centerX = (gate.start.x + gate.end.x) / 2;

    int gateY = gate.start.y;

    passes.push_back({ { centerX, gateY - 1 }, { centerX, gateY + 1 }, Direction::DOWN });

    passes.push_back({ { centerX, gateY + 1 }, { centerX, gateY - 1 }, Direction::UP });

    return passes;
  }

  // ==========================================
  // 縦向きゲート
  //
  //        |
  //        |
  //
  // (8,4) → ← (6,4)
  //
  // RIGHTはX減少
  // LEFTはX増加
  // ==========================================

  int centerY = (gate.start.y + gate.end.y) / 2;

  int gateX = gate.start.x;

  // Xが大きい側 → Xが小さい側
  // RIGHT
  passes.push_back({ { gateX + 1, centerY }, { gateX - 1, centerY }, Direction::RIGHT });

  // Xが小さい側 → Xが大きい側
  // LEFT
  passes.push_back({ { gateX - 1, centerY }, { gateX + 1, centerY }, Direction::LEFT });

  return passes;
}