/**
 * @file   SquareDetectionActionHandler.h
 * @brief  正方形検出要求を処理し、
 *         画像座標を実距離へ変換するクラス
 * @author okuyama0528 yutaro-1214
 */

#ifndef SQUARE_DETECTION_ACTION_HANDLER_H
#define SQUARE_DETECTION_ACTION_HANDLER_H

#include <opencv2/opencv.hpp>

#include "CameraCapture.h"
#include "Logger.h"
#include "SocketProtocol.h"
#include "SquareDetector.h"

class SquareDetectionActionHandler {
 public:
  /**
   * @brief コンストラクタ
   * @param _camera カメラ
   */
  explicit SquareDetectionActionHandler(CameraCapture& _camera);

  /**
   * @brief デストラクタ
   */
  ~SquareDetectionActionHandler();

  /**
   * @brief 正方形検出要求を処理する
   *
   * 正方形を検出した後、ホモグラフィ変換によって
   * カメラ基準の前方距離・横方向距離を計算する。
   *
   * @param request 正方形検出リクエスト
   * @param response 正方形検出レスポンス
   */
  void execute(
      const CameraServer::SquareDetectorRequest& request,
      CameraServer::SquareDetectorResponse& response);

 private:
  /**
   * @brief カメラ
   */
  CameraCapture& camera;

  /**
   * @brief 正方形検出器
   */
  SquareDetector detector;

  /**
   * @brief 画像座標から校正用紙上の実座標へ変換する
   *        ホモグラフィ行列
   */
  cv::Mat homography;

  /**
   * @brief 8個の基準点からホモグラフィ行列を初期化する
   */
  void initializeHomography();

  /**
   * @brief 画像座標を校正用紙上の実座標へ変換する
   *
   * 校正用紙左下を原点とし、
   * 右方向をXプラス、奥方向をYプラスとする。
   *
   * @param pixelX 画像X座標[px]
   * @param pixelY 画像Y座標[px]
   * @return 校正用紙上の座標[mm]
   */
  cv::Point2f pixelToWorld(
      double pixelX,
      double pixelY) const;
};

#endif  // SQUARE_DETECTION_ACTION_HANDLER_H