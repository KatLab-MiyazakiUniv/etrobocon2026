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
   */
  struct Node {
    int gridX;  // 格子X座標
    int gridY;  // 格子Y座標
    double x;   // 実X座標[mm]
    double y;   // 実Y座標[mm]
  };

  /**
   * @brief コンストラクタ
   */
  EtRallyMap();

  /**
   * @brief 指定した格子座標の交点情報を取得する
   * @param gridX 格子X座標
   * @param gridY 格子Y座標
   * @return 指定した交点情報
   */
  Node getNode(int gridX, int gridY) const;

  /**
   * @brief X方向の最初の区間を設定する
   * @param value X方向の最初の区間[mm]
   */
  void setFirstXGridSize(double value);

  /**
   * @brief X方向の通常区間を設定する
   * @param value X方向の通常区間[mm]
   */
  void setGridSizeX(double value);

  /**
   * @brief Y方向の最初の区間を設定する
   * @param value Y方向の最初の区間[mm]
   */
  void setFirstYGridSize(double value);

  /**
   * @brief Y方向の通常区間を設定する
   * @param value Y方向の通常区間[mm]
   */
  void setGridSizeY(double value);

  /**
   * @brief Y方向の最後の区間を設定する
   * @param value Y方向の最後の区間[mm]
   */
  void setLastYGridSize(double value);

 private:
  /**
   * @brief 全ノードの実座標を再計算する
   */
  void updateNodes();

  // X方向の最初の区間
  double firstXGridSize = SystemInfo::FIRST_X_GRID_SIZE;

  // X方向の通常区間
  double gridSizeX = SystemInfo::GRID_SIZE_X;

  // Y方向の最初の区間
  double firstYGridSize = SystemInfo::FIRST_Y_GRID_SIZE;

  // Y方向の通常区間
  double gridSizeY = SystemInfo::GRID_SIZE_Y;

  // Y方向の最後の区間
  double lastYGridSize = SystemInfo::LAST_Y_GRID_SIZE;

  // X方向の区間数
  static constexpr int X_GRID_NUM = SystemInfo::X_GRID_NUM;

  // Y方向の区間数
  static constexpr int Y_GRID_NUM = SystemInfo::Y_GRID_NUM;

  /**
   * @brief マップ上の交点情報を保持する2次元配列
   *
   * gridXが増えるほど左へ進む。
   * gridYが増えるほど下へ進む。
   *
   * X_GRID_NUM、Y_GRID_NUMは区間数なので、
   * 交点数はそれぞれ+1となる。
   */
  Node nodes[X_GRID_NUM + 1][Y_GRID_NUM + 1];
};

#endif  // ET_RALLY_MAP_H