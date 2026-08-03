/**
 * @file   EtRallyMap.h
 * @brief  ETラリーのマップ情報を管理するクラス
 * @author okuyama0528
 */

#ifndef ET_RALLY_MAP_H
#define ET_RALLY_MAP_H

class EtRallyMap {
 public:
  /**
   * @brief 右上を原点とした交点情報
   */
  struct Node {
    int gridX;     // 格子座標X
    int gridY;     // 格子座標Y

    double x;      // 実座標X(mm)
    double y;      // 実座標Y(mm)
  };

  /**
   * @brief コンストラクタ
   */
  EtRallyMap();

  /**
   * @brief 指定した格子座標の交点情報を取得する
   * @param gridX X方向の格子番号
   * @param gridY Y方向の格子番号
   * @return 指定位置の交点情報(Node)
   */
  Node getNode(int gridX, int gridY) const;

 private:
  // X方向の最初の区間(mm)
  static constexpr double FIRST_X_GRID_SIZE = 171.0;

  // X方向の通常の区間(mm)
  static constexpr double GRID_SIZE = 104.0;

  // Y方向の全長(mm)
  static constexpr double MAP_HEIGHT = 1135.0;

  // Y方向の区間(mm)
  static constexpr double GRID_SIZE_Y = MAP_HEIGHT / 10.0;

  // 格子数（原点を含めて11本）
  static constexpr int X_GRID_NUM = 10;
  static constexpr int Y_GRID_NUM = 10;

  Node nodes[X_GRID_NUM + 1][Y_GRID_NUM + 1];
};

#endif