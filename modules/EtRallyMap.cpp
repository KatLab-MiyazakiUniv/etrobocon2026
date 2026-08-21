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

      // =====================================================
      // X方向の実座標
      // =====================================================
      //
      // gridX=0 → 1 の区間のみ firstXGridSize
      //
      // それ以降は gridSizeX
      //
      // 例:
      //
      // 0 ---- firstXGridSize ---- 1 ---- gridSizeX ---- 2
      //                               ---- gridSizeX ---- 3
      //
      // =====================================================

      if(gridX == 0) {
        nodes[gridX][gridY].x = 0.0;

      } else if(gridX == 1) {
        nodes[gridX][gridY].x = firstXGridSize;

      } else {
        nodes[gridX][gridY].x = firstXGridSize + (gridX - 1) * gridSizeX;
      }

      // =====================================================
      // Y方向の実座標
      // =====================================================
      //
      // 通常区間は gridSizeY
      //
      // 最後の区間
      //
      // Y_GRID_NUM - 1 → Y_GRID_NUM
      //
      // のみ lastYGridSize
      //
      // Y方向はgridYが増えるほど負方向へ進む。
      //
      // =====================================================

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
  gridSizeX = value;

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