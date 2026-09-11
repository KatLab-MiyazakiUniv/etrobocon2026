/**
 * @file   SquareAngleAdjustment.h
 * @brief  正方形の画像座標からロボット基準の角度・距離を計算するクラス
 * @author yutaro-1214
 */

#ifndef SQUARE_ANGLE_ADJUSTMENT_H
#define SQUARE_ANGLE_ADJUSTMENT_H

#include "Robot.h"
#include "SocketProtocol.h"

/**
 * @brief 正方形検出結果から補正角度と距離を計算するクラス
 *
 * 正方形の中心座標をカメラ画像から取得し、
 *
 * 画像座標
 *   (x, y)
 *
 * を
 *
 * ロボット基準座標
 *   前方距離[mm]
 *   横方向距離[mm]
 *
 * に変換する。
 *
 * その後、
 *
 *   補正角度 = atan2(横方向距離, 前方距離)
 *
 *   直進距離 = sqrt(前方距離^2 + 横方向距離^2)
 *
 * を計算する。
 *
 * このクラス自身はモータを動かさない。
 */
class SquareAngleAdjustment {
 public:
  /**
   * @brief 正方形補正計算結果
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
     * @brief タイヤ軸から正方形までの前方距離[mm]
     */
    double forwardDistance = 0.0;

    /**
     * @brief ロボット中心から正方形までの横方向距離[mm]
     *
     * 負：左
     * 正：右
     */
    double lateralDistance = 0.0;

    /**
     * @brief 正方形方向へ向くための相対回頭角度[deg]
     */
    double correctionAngle = 0.0;

    /**
     * @brief 回頭後に正方形まで直進する距離[mm]
     */
    double straightDistance = 0.0;
  };

  /**
   * @brief コンストラクタ
   * @param _robot ロボットインスタンス
   */
  explicit SquareAngleAdjustment(Robot& _robot);

  /**
   * @brief デストラクタ
   */
  ~SquareAngleAdjustment();

  /**
   * @brief 正方形を検出して補正値を計算する
   *
   * @param request 正方形検出リクエスト
   * @return 計算結果
   */
  Result calculate(const CameraServer::SquareDetectorRequest& request);

 private:
  /**
   * @brief ロボットインスタンス
   */
  Robot& robot;

  /**
   * @brief 画像Y座標から前方距離を計算する
   *
   * @param y 画像Y座標[px]
   * @return タイヤ軸からの前方距離[mm]
   */
  double pixelYToForwardDistance(double y) const;

  /**
   * @brief 画像座標から横方向距離を計算する
   *
   * @param x 画像X座標[px]
   * @param y 画像Y座標[px]
   * @return 横方向距離[mm]
   */
  double pixelToLateralDistance(double x, double y) const;

  /**
   * @brief 前方距離と横方向距離から回頭角度を計算する
   *
   * @param forwardDistance 前方距離[mm]
   * @param lateralDistance 横方向距離[mm]
   * @return 相対回頭角度[deg]
   */
  double calculateCorrectionAngle(double forwardDistance, double lateralDistance) const;
};

#endif