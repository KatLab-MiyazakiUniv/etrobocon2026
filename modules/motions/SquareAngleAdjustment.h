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
  struct Result {
    bool wasDetected = false;

    double centerX = 0.0;
    double centerY = 0.0;

    double forwardDistance = 0.0;

    double lateralDistance = 0.0;

    double correctionAngle = 0.0;

    double straightDistance = 0.0;
  };

  explicit SquareAngleAdjustment(
      Robot& _robot);

  ~SquareAngleAdjustment();

  Result calculate(
      const CameraServer::SquareDetectorRequest& request);

 private:
  Robot& robot;

  double pixelYToForwardDistance(
      double y) const;

  double pixelToLateralDistance(
      double x,
      double y) const;

  double calculateCorrectionAngle(
      double forwardDistance,
      double lateralDistance) const;
};

#endif