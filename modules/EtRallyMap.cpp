/**
 * @file   EtRallyMap.cpp
 * @brief  ETラリーのマップ情報を管理するクラス
 * @author okuyama0528
 */

#include "EtRallyMap.h"

EtRallyMap::EtRallyMap()
{
  for(int x = 0; x <= X_GRID_NUM; x++) {
    for(int y = 0; y <= Y_GRID_NUM; y++) {
      nodes[x][y].gridX = x;
      nodes[x][y].gridY = y;

      // X座標
      if(x == 0) {
        nodes[x][y].x = 0;
      } else if(x == 1) {
        nodes[x][y].x = FIRST_X_GRID_SIZE;
      } else {
        nodes[x][y].x = FIRST_X_GRID_SIZE + (x - 1) * GRID_SIZE;
      }

      // Y座標
      if(y < Y_GRID_NUM) {
        nodes[x][y].y = y * GRID_SIZE;
      } else {
        nodes[x][y].y = (Y_GRID_NUM - 1) * GRID_SIZE + LAST_Y_GRID_SIZE;
      }
    }
  }
}

EtRallyMap::Node EtRallyMap::getNode(int gridX, int gridY) const
{
  return nodes[gridX][gridY];
}