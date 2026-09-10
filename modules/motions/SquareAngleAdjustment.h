/**
 * @file   SquareAngleAdjustment.h
 * @brief  正方形の中心を利用してロボットの向きを補正するクラス
 * @author yutaro-1214
 */

#ifndef SQUARE_ANGLE_ADJUSTMENT_H
#define SQUARE_ANGLE_ADJUSTMENT_H

#include "Pid.h"
#include "Robot.h"
#include "SocketProtocol.h"

class SquareAngleAdjustment {
 public:
  SquareAngleAdjustment(
      Robot& _robot,
      const CameraServer::SquareDetectorRequest& _squareDetectionRequest,
      const Pid::PidGain& _pidGain,
      double _centerTolerance = 10.0);

  ~SquareAngleAdjustment();

  /**
   * @brief 正方形の中心が画像中央に来るように補正する
   *
   * @return true 補正成功
   * @return false 正方形未検出または通信失敗
   */
  bool run();

 private:
  /**
   * @brief 正方形中心と画像中心のX方向誤差を計算する
   *
   * 右なら正、左なら負を返す。
   *
   * @param response 正方形検出結果
   * @return X方向誤差[pixel]
   */
  double calculateCenterError(
      const CameraServer::SquareDetectorResponse& response) const;

  /**
   * @brief 左右モータを停止する
   */
  void stop();

 private:
  Robot& robot;

  CameraServer::SquareDetectorRequest squareDetectionRequest;

  Pid centerPid;

  /**
   * @brief 中心補正終了許容誤差[pixel]
   */
  double centerTolerance;
};

#endif