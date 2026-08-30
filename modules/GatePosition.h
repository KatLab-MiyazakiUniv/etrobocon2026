/**
 * @file   GatePosition.h
 * @brief  ゲート位置情報を管理するクラス
 * @author migaku2645
 */

#ifndef GATE_POSITION_H
#define GATE_POSITION_H

#include "Logger.h"
/**
 * @brief ゲートの色
 */
enum class GateColor { RED, BLUE, YELLOW };

/**
 * @brief ゲート位置情報を管理するクラス
 */
class GatePosition {
 public:
  /**
   * @brief コンストラクタ
   */
  GatePosition();

  /**
   * @brief ゲート位置を設定する
   * @param color ゲートの色
   * @param position1 1つ目の位置情報
   * @param position2 2つ目の位置情報
   */
  void setPosition(GateColor color, int position1, int position2);

  /**
   * @brief ゲート位置を取得する
   * @param color ゲートの色
   * @param position 位置情報を格納する配列
   */
  void getPosition(GateColor color, int position[2]) const;

 private:
  int red[2];
  int blue[2];
  int yellow[2];
};

#endif