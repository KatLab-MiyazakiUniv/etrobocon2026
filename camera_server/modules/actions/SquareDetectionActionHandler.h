/**
 * @file   SquareDetectionActionHandler.h
 * @brief  正方形検出要求を処理するクラス
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
   *
   * @param _camera カメラ
   */
  explicit SquareDetectionActionHandler(
      CameraCapture& _camera);

  /**
   * @brief デストラクタ
   */
  ~SquareDetectionActionHandler();

  /**
   * @brief 正方形検出要求を処理する
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
};

#endif