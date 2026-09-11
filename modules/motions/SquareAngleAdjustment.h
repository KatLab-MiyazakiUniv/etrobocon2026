/**
 * @file   SquareAngleAdjustment.h
 * @brief  正方形の画像座標からロボット基準の角度・距離を計算するクラス
 * @author yutaro-1214
 */

#ifndef SQUARE_ANGLE_ADJUSTMENT_H
#define SQUARE_ANGLE_ADJUSTMENT_H

#include "Robot.h"
#include "SocketProtocol.h"

class SquareAngleAdjustment {
 public:
  /**
   * @brief 正方形検出・位置計算結果
   */
  struct Result {
    /**
     * @brief 正方形を検出できたか
     */
    bool wasDetected = false;

    /**
     * @brief 正方形中心X座標[px]
     */
    double centerX = 0.0;

    /**
     * @brief 正方形中心Y座標[px]
     */
    double centerY = 0.0;

    /**
     * @brief ロボットから見た前方距離[mm]
     */
    double forwardDistance = 0.0;

    /**
     * @brief ロボットから見た横方向距離[mm]
     *
     * 負: 左
     * 正: 右
     */
    double lateralDistance = 0.0;

    /**
     * @brief 正方形へ向くための補正角度[deg]
     */
    double correctionAngle = 0.0;

    /**
     * @brief 正方形までの直線距離[mm]
     */
    double straightDistance = 0.0;
  };

  /**
   * @brief コンストラクタ
   */
  explicit SquareAngleAdjustment(Robot& _robot);

  /**
   * @brief デストラクタ
   */
  ~SquareAngleAdjustment();

  /**
   * @brief 正方形を検出し、
   *        角度・距離を計算する
   *
   * @param request 正方形検出リクエスト
   * @return 計算結果
   */
  Result calculate(const CameraServer::SquareDetectorRequest& request);

 private:
  /**
   * @brief ロボット
   */
  Robot& robot;

  /**
   * @brief 画像Y座標を前方距離へ変換する
   */
  double pixelYToForwardDistance(double y) const;

  /**
   * @brief 画像X,Y座標を横方向距離へ変換する
   */
  double pixelToLateralDistance(double x, double y) const;

  /**
   * @brief 前方距離・横距離から補正角度を計算する
   */
  double calculateCorrectionAngle(double forwardDistance, double lateralDistance) const;
};

#endif