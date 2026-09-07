/**
 * @file   SquareAngleAdjustment.h
 * @brief  正方形を利用してロボットの向きを補正するクラス
 * @author yutaro-1214
 */

#ifndef SQUARE_ANGLE_ADJUSTMENT_H
#define SQUARE_ANGLE_ADJUSTMENT_H

#include "Pid.h"
#include "Robot.h"
#include "SocketProtocol.h"

/**
 * @class SquareAngleAdjustment
 * @brief 正方形の傾きを利用してロボットの向きを補正するクラス
 */
class SquareAngleAdjustment {
 public:
  /**
   * @brief コンストラクタ
   *
   * @param _robot ロボット
   * @param _squareDetectionRequest 正方形検出リクエスト
   * @param _pidGain 角度補正PID
   * @param _angleTolerance 角度補正終了許容誤差[deg]
   */
  SquareAngleAdjustment(Robot& _robot,
                        const CameraServer::SquareDetectorRequest& _squareDetectionRequest,
                        const Pid::PidGain& _pidGain, double _angleTolerance = 2.0);

  /**
   * @brief デストラクタ
   */
  ~SquareAngleAdjustment();

  /**
   * @brief 正方形を利用して角度補正する
   *
   * 正方形を検出できなかった場合は
   * その回の補正を行わずfalseを返す。
   *
   * @return true 補正成功
   * @return false 正方形未検出または通信失敗
   */
  bool run();

 private:
  /**
   * @brief 正方形の上辺の傾きを計算する
   *
   * @param response 正方形検出結果
   * @return 正方形の傾き[deg]
   */
  double calculateSquareAngle(const CameraServer::SquareDetectorResponse& response) const;

  /**
   * @brief 左右モータを停止する
   */
  void stop();

  /**
   * @brief ロボット
   */
  Robot& robot;

  /**
   * @brief 正方形検出リクエスト
   */
  CameraServer::SquareDetectorRequest squareDetectionRequest;

  /**
   * @brief 角度補正PID
   */
  Pid anglePid;

  /**
   * @brief 角度補正終了許容誤差[deg]
   */
  double angleTolerance;
};

#endif