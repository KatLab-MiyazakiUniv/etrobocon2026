/**
 * @file   EtRallyMap.cpp
 * @brief  ETラリーのマップ情報を管理するクラス
 * @author okuyama0528
 */

#include "EtRallyMap.h"

EtRallyMap::EtRallyMap()
{
  updateNodes();
}

void EtRallyMap::updateNodes()
{
  for(int gridX = 0; gridX <= X_GRID_NUM; ++gridX) {
    for(int gridY = 0; gridY <= Y_GRID_NUM; ++gridY) {
      nodes[gridX][gridY].gridX = gridX;
      nodes[gridX][gridY].gridY = gridY;

      // X方向
      if(gridX == 0) {
        nodes[gridX][gridY].x = 0.0;
      } else if(gridX == 1) {
        nodes[gridX][gridY].x = firstXGridSize;
      } else {
        nodes[gridX][gridY].x = firstXGridSize + (gridX - 1) * gridSize;
      }

      // Y方向（最後の区間だけ長さを変更）
      if(gridY == Y_GRID_NUM) {
        nodes[gridX][gridY].y = -((Y_GRID_NUM - 1) * gridSizeY + lastYGridSize);
      } else {
        nodes[gridX][gridY].y = -gridY * gridSizeY;
      }
    }
  }
}

EtRallyMap::Node EtRallyMap::getNode(int gridX, int gridY) const
{
  return nodes[gridX][gridY];
}

void EtRallyMap::setFirstXGridSize(double value)
{
  firstXGridSize = value;

  updateNodes();
}

void EtRallyMap::setGridSizeX(double value)
{
  gridSize = value;

  updateNodes();
}

void EtRallyMap::setGridSizeY(double value)
{
  gridSizeY = value;

  updateNodes();
}