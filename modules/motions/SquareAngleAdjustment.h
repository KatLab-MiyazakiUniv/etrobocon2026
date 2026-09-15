/**
 * @file   SquareAngleAdjustment.h
 * @brief  正方形の画像座標をホモグラフィ変換して、
 *         カメラ基準の実距離・補正角度を求めるクラス
 * @author yutaro-1214
 */

#ifndef SQUARE_ANGLE_ADJUSTMENT_H
#define SQUARE_ANGLE_ADJUSTMENT_H

#include <opencv2/opencv.hpp>

#include "Robot.h"
#include "SocketProtocol.h"

class SquareAngleAdjustment {
 public:
  /**
   * @brief 正方形検出・座標変換結果
   */
  struct Result {
    /**
     * @brief 正方形を検出できたか
     */
    bool wasDetected = false;

    /**
     * @brief 画像上の正方形中心X座標[px]
     */
    double centerX = 0.0;

    /**
     * @brief 画像上の正方形中心Y座標[px]
     */
    double centerY = 0.0;

    /**
     * @brief 校正用紙上のX座標[mm]
     *
     * 左上を原点とする。
     */
    double worldX = 0.0;

    /**
     * @brief 校正用紙上のY座標[mm]
     *
     * 左上を原点とする。
     */
    double worldY = 0.0;

    /**
     * @brief カメラ直下の床点から見た前方距離[mm]
     */
    double forwardDistance = 0.0;

    /**
     * @brief カメラ正面中心から見た横方向距離[mm]
     *
     * 左: 負
     * 右: 正
     */
    double lateralDistance = 0.0;

    /**
     * @brief 正方形へ向くための補正角度[deg]
     */
    double correctionAngle = 0.0;

    /**
     * @brief カメラ直下基準から正方形までの直線距離[mm]
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
   *        画像座標を実座標へ変換する
   *
   * @param request 正方形検出要求
   * @return 検出・変換結果
   */
  Result calculate(const CameraServer::SquareDetectorRequest& request);

 private:
  /**
   * @brief ロボット
   */
  Robot& robot;

  /**
   * @brief 画像座標から実座標へのホモグラフィ行列
   */
  cv::Mat homography;

  /**
   * @brief ホモグラフィ行列を初期化する
   */
  void initializeHomography();

  /**
   * @brief 画像座標を校正用紙上のmm座標へ変換する
   */
  cv::Point2f pixelToWorld(double pixelX, double pixelY) const;

  /**
   * @brief 前方距離と横方向距離から補正角度を求める
   */
  double calculateCorrectionAngle(double forwardDistance, double lateralDistance) const;
};

#endif