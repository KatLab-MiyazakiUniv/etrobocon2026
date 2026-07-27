/**
 * @file   Odometry.h
 * @brief  移動距離と向いている方向からPositionを更新するクラス
 * @author yutaro-1214
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <cstdint>

#include "Position.h"
#include "SystemInfo.h"

class Odometry {
 public:
  /**
   * @brief コンストラクタ
   * @param position 更新対象
   */
  explicit Odometry(Position& position);

  /**
   * @brief Positionをリセット
   */
  void reset();

  /**
   * @brief エンコード値をセット
   * @param encoderLeft 左エンコーダ値[deg]
   * @param encoderRight 右エンコーダ値[deg]
   */
  void initialize(int32_t encoderLeft, int32_t encoderRight);
  /**
   * @brief エンコーダとIMUから自己位置を更新
   * @param encoderLeft 左エンコーダ値[deg]
   * @param encoderRight 右エンコーダ値[deg]
   * @param heading IMU方位角[deg]
   */
  void update(int32_t encoderLeft, int32_t encoderRight, double heading);

 private:
  Position& position;  // 更新対象となるロボットの位置情報

  int32_t prevEncoderLeft;   // 前回取得した左車輪のエンコーダ値
  int32_t prevEncoderRight;  // 前回取得した右車輪のエンコーダ値
};

#endif