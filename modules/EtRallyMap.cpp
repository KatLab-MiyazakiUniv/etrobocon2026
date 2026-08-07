/**
 * @file   EtRallyMap.cpp
 * @brief  ETラリーのマップ情報を管理するクラス
 * @author okuyama0528
 */

#include "EtRallyMap.h"

EtRallyMap::EtRallyMap()
{
  for(int x = 0; x <= X_GRID_NUM; x++) {
    for(int y = 0; y <= Y_GRID_NUM + 1; y++) {
      // 格子座標
      nodes[x][y].gridX = x;
      nodes[x][y].gridY = y - 1;

      // X座標
      if(x == 0) {
        nodes[x][y].x = 0.0;
      } else if(x == 1) {
        nodes[x][y].x = firstXGridSize;
      } else {
        nodes[x][y].x = firstXGridSize + (x - 1) * gridSize;
      }

      // Y座標
      nodes[x][y].y = (y - 1) * gridSizeY;
    }
  }
}

EtRallyMap::Node EtRallyMap::getNode(int gridX, int gridY) const
{
  return nodes[gridX][gridY + 1];
}

void EtRallyMap::setFirstXGridSize(double value)
{
  firstXGridSize = value;
}

void EtRallyMap::setGridSize(double value)
{
  gridSize = value;
}

void EtRallyMap::setGridSizeY(double value)
{
  gridSizeY = value;
}