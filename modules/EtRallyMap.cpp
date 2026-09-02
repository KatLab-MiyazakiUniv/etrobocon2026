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
      // 格子座標を設定
      nodes[gridX][gridY].gridX = gridX;
      nodes[gridX][gridY].gridY = gridY;

      if(gridX == 0) {
        nodes[gridX][gridY].x = 0.0;

      } else if(gridX == 1) {
        nodes[gridX][gridY].x = firstXGridSize;

      } else if(gridX == X_GRID_NUM) {
        nodes[gridX][gridY].x = firstXGridSize + (X_GRID_NUM - 2) * gridSizeX + lastXGridSize;

      } else {
        nodes[gridX][gridY].x = firstXGridSize + (gridX - 1) * gridSizeX;
      }

      if(gridY == 0) {
        nodes[gridX][gridY].y = 0.0;

      } else if(gridY == 1) {
        nodes[gridX][gridY].y = -firstYGridSize;

      } else if(gridY == Y_GRID_NUM) {
        nodes[gridX][gridY].y = -(firstYGridSize + (Y_GRID_NUM - 2) * gridSizeY + lastYGridSize);

      } else {
        nodes[gridX][gridY].y = -(firstYGridSize + (gridY - 1) * gridSizeY);
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
  gridSizeX = value;

  updateNodes();
}

void EtRallyMap::setLastXGridSize(double value)
{
  lastXGridSize = value;

  updateNodes();
}

void EtRallyMap::setFirstYGridSize(double value)
{
  firstYGridSize = value;

  updateNodes();
}

void EtRallyMap::setGridSizeY(double value)
{
  gridSizeY = value;

  updateNodes();
}

void EtRallyMap::setLastYGridSize(double value)
{
  lastYGridSize = value;

  updateNodes();
}