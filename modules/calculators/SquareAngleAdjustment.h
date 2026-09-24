/**
 * @file   SquareAngleAdjustment.h
 * @brief  正方形検出結果からロボットの補正角度・距離を計算するクラス
 * @author yutaro-1214
 */

#ifndef SQUARE_ANGLE_ADJUSTMENT_H
#define SQUARE_ANGLE_ADJUSTMENT_H

#include "Robot.h"
#include "SocketProtocol.h"
#include "SocketClient.h"
#include "Logger.h"
#include <cmath>

class SquareAngleAdjustment {
 public:
  /**
   * @brief 計算結果を保持する構造体
   */
  struct Result {
    bool wasDetected = false;       // 正方形を検出したか
    double centerX = 0.0;           // 正方形中心X座標[px]
    double centerY = 0.0;           // 正方形中心Y座標[px]
    double forwardDistance = 0.0;   // 前方距離[mm]
    double lateralDistance = 0.0;   // 横方向距離[mm]
    double correctionAngle = 0.0;   // 補正角度[deg]
    double straightDistance = 0.0;  // 直線距離[mm]
  };

  /**
   * @brief コンストラクタ
   * @param _robot Robotクラスのインスタンス
   */
  explicit SquareAngleAdjustment(Robot& _robot);

  /**
   * @brief デストラクタ
   */
  ~SquareAngleAdjustment();

  /**
   * @brief 正方形の検出結果から補正角度と距離を計算する
   * @param request カメラサーバーへの検出要求
   * @return 計算結果
   */
  Result calculate(const CameraServer::SquareDetectorRequest& request);

 private:
  Robot& robot;  // Robotクラスのインスタンス

  /**
   * @brief 前方距離と横方向距離から補正角度を計算する
   * @param forwardDistance 前方距離[mm]
   * @param lateralDistance 横方向距離[mm]
   * @return 補正角度[deg]
   */
  double calculateCorrectionAngle(double forwardDistance, double lateralDistance) const;
};

#endif  // SQUARE_ANGLE_ADJUSTMENT_H