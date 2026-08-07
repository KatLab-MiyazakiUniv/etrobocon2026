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
   * @brief 右上のY座標一つ下を原点とした交点情報
   */
  struct Node {
    int gridX;
    int gridY;  // -1～9

    double x;
    double y;
  };

  /**
   * @brief コンストラクタ
   */
  EtRallyMap();

  /**
   * @brief 指定した交点情報を取得
   */
  Node getNode(int gridX, int gridY) const;

  /**
   * @brief マップ寸法の設定
   * @param value X方向の最初の区間(mm)
   */
  void setFirstXGridSize(double value);

  /**
   * @brief マップ寸法の設定
   * @param value X方向の通常の区間(mm)
   */
  void setGridSize(double value);

  /**
   * @brief マップ寸法の設定
   * @param value Y方向の区間(mm)
   */
  void setGridSizeY(double value);

 private:
  // SystemInfoの値を初期値として保持
  double firstXGridSize = SystemInfo::FIRST_X_GRID_SIZE;     // X方向の最初の区間(mm)
  double gridSize = SystemInfo::GRID_SIZE;                   // X方向の通常の区間(mm)
  double gridSizeY = SystemInfo::GRID_SIZE_Y;                // Y方向の区間(mm)
  static constexpr int X_GRID_NUM = SystemInfo::X_GRID_NUM;  // X方向の区間数
  static constexpr int Y_GRID_NUM = SystemInfo::Y_GRID_NUM;  // Y方向の区間数（-1～9を使用）

  // gridY=-1～9なので+2
  Node nodes[X_GRID_NUM + 1][Y_GRID_NUM + 2];  // 交点情報
};

#endif