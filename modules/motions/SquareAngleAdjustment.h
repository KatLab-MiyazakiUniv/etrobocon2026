/**
 * @file   SquareAngleAdjustment.h
 * @brief  正方形の中心位置から角度を計算し、RelativeRotationで向きを補正するクラス
 * @author yutaro-1214
 */

#ifndef SQUARE_ANGLE_ADJUSTMENT_H
#define SQUARE_ANGLE_ADJUSTMENT_H

#include "Pid.h"
#include "Robot.h"
#include "SocketProtocol.h"

class SquareAngleAdjustment {
 public:
  /**
   * @brief コンストラクタ
   *
   * @param _robot ロボット
   * @param _squareDetectionRequest 正方形検出リクエスト
   * @param _rotationPid RelativeRotationで使用するPID
   * @param _centerTolerance 中央とみなす許容ピクセル差
   * @param _horizontalFovDeg カメラの水平画角[deg]
   * @param _rotationTolerance RelativeRotationの終了許容角度[deg]
   */
  SquareAngleAdjustment(
      Robot& _robot,
      const CameraServer::SquareDetectorRequest& _squareDetectionRequest,
      const Pid::PidGain& _rotationPid,
      double _centerTolerance = 5.0,
      double _horizontalFovDeg = 70.0,
      double _rotationTolerance = 1.0);

  /**
   * @brief デストラクタ
   */
  ~SquareAngleAdjustment();

  /**
   * @brief 正方形を検出して角度補正を行う
   *
   * @return true 補正成功または既に中央にある
   * @return false 正方形を検出できなかった
   */
  bool run();

 private:
  /**
   * @brief 正方形の中心X座標を求める
   *
   * @param response 正方形検出結果
   * @return double 中心X座標[pixel]
   */
  double calculateCenterX(
      const CameraServer::SquareDetectorResponse& response) const;

  /**
   * @brief 画像中心からのピクセル誤差を求める
   *
   * @param response 正方形検出結果
   * @return double ピクセル誤差
   *
   * 正:
   *   正方形が画像中心より右
   *
   * 負:
   *   正方形が画像中心より左
   */
  double calculatePixelError(
      const CameraServer::SquareDetectorResponse& response) const;

  /**
   * @brief ピクセル誤差をRelativeRotation用の角度へ変換する
   *
   * @param pixelError 画像中心からのピクセル差
   * @return double 補正角度[deg]
   */
  double pixelErrorToRotationAngle(
      double pixelError) const;

  /**
   * @brief RelativeRotationで角度補正する
   *
   * @param angle 補正角度[deg]
   */
  void rotate(
      double angle);

  /**
   * @brief 左右モータを停止する
   */
  void stop();

 private:
  /**
   * @brief ロボット
   */
  Robot& robot;

  /**
   * @brief 正方形検出リクエスト
   */
  CameraServer::SquareDetectorRequest squareDetectionRequest;

  /**
   * @brief RelativeRotation用PID
   */
  Pid::PidGain rotationPid;

  /**
   * @brief 正方形中心の許容誤差[pixel]
   */
  double centerTolerance;

  /**
   * @brief カメラ水平画角[deg]
   */
  double horizontalFovDeg;

  /**
   * @brief RelativeRotation終了許容角度[deg]
   */
  double rotationTolerance;
};

#endif  // SQUARE_ANGLE_ADJUSTMENT_H