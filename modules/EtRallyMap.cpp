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
  /*
   * gridX : 0～11
   *
   * 右端が0
   * 左へ行くほど増加
   */
  for(int gridX = 0; gridX <= X_GRID_NUM; ++gridX) {
    /*
     * gridY : 0～11
     *
     * 上端が0
     * 下へ行くほど増加
     */
    for(int gridY = 0; gridY <= Y_GRID_NUM + 1; ++gridY) {
      // =========================
      // 格子座標
      // =========================

      nodes[gridX][gridY].gridX = gridX;

      nodes[gridX][gridY].gridY = gridY;

      // =========================
      // 実X座標
      // =========================

      /*
       * gridX = 0
       * → x = 0 mm
       *
       * gridX = 1
       * → x = 175 mm
       *
       * gridX = 2
       * → x = 297.5 mm
       *
       * gridXが増えるほど
       * 実X座標も増加する。
       *
       * したがって、
       * 左へ行くほどXが増える。
       */

      if(gridX == 0) {
        nodes[gridX][gridY].x = 0.0;

      } else if(gridX == 1) {
        nodes[gridX][gridY].x = firstXGridSize;

      } else {
        nodes[gridX][gridY].x = firstXGridSize + (gridX - 1) * gridSize;
      }

      // =========================
      // 実Y座標
      // =========================

      /*
       * gridY = 0
       * → y = 0 mm
       *
       * gridY = 1
       * → y = -122.5 mm
       *
       * gridY = 2
       * → y = -245.0 mm
       *
       * 下へ行くほど
       * 実Y座標はマイナスになる。
       */

      nodes[gridX][gridY].y = -gridY * gridSizeY;
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

void EtRallyMap::setGridSize(double value)
{
  gridSize = value;

  updateNodes();
}

void EtRallyMap::setGridSizeY(double value)
{
  gridSizeY = value;

  updateNodes();
}