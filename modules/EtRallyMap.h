/**
 * @file   EtRallyMap.h
 * @brief  ETラリーのマップ情報を管理するクラス
 * @author okuyama0528
 */

#ifndef ET_RALLY_MAP_H
#define ET_RALLY_MAP_H

#include "SystemInfo.h"

class EtRallyMap {
 public:
  /**
   * @brief マップ上の交点情報
   *
   * gridX:
   *   右端を0とし、左へ行くほど増加する
   *
   * gridY:
   *   上端を0とし、下へ行くほど増加する
   */
  struct Node {
    int gridX;
    int gridY;

    double x;
    double y;
  };

  /**
   * @brief コンストラクタ
   */
  EtRallyMap();

  /**
   * @brief 指定した格子座標の交点情報を取得する
   *
   * @param gridX 格子X座標
   * @param gridY 格子Y座標
   *
   * @return 指定した交点情報
   */
  Node getNode(int gridX, int gridY) const;

  /**
   * @brief X方向の最初の区間を設定する
   *
   * @param value X方向の最初の区間[mm]
   */
  void setFirstXGridSize(double value);

  /**
   * @brief X方向の通常区間を設定する
   *
   * @param value X方向の通常区間[mm]
   */
  void setGridSize(double value);

  /**
   * @brief Y方向の区間を設定する
   *
   * @param value Y方向の区間[mm]
   */
  void setGridSizeY(double value);

 private:
  /**
   * @brief 全ノードの実座標を計算する
   */
  void updateNodes();

  // SystemInfoの値を初期値として保持
  double firstXGridSize = SystemInfo::FIRST_X_GRID_SIZE;

  double gridSize = SystemInfo::GRID_SIZE;

  double gridSizeY = SystemInfo::GRID_SIZE_Y;

  // X方向の区間数
  static constexpr int X_GRID_NUM = SystemInfo::X_GRID_NUM;

  // 元々のY方向区間数
  static constexpr int Y_GRID_NUM = SystemInfo::Y_GRID_NUM;

  /*
   * X : 0～11 → 12ノード
   * Y : 0～11 → 12ノード
   */
  Node nodes[X_GRID_NUM + 1][Y_GRID_NUM + 2];
};

#endif  // ET_RALLY_MAP_H