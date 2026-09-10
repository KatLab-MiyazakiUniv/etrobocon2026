/**
 * @file SquareDetectionActionHandler.h
 * @brief 正方形検出処理を実行するActionHandler
 */

#ifndef SQUARE_DETECTION_ACTION_HANDLER_H
#define SQUARE_DETECTION_ACTION_HANDLER_H

#include <memory>

#include "CameraCapture.h"
#include "SocketProtocol.h"
#include "SquareDetector.h"

/**
 * @brief カメラ画像から正方形を検出するActionHandler
 */
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
   * @brief 正方形検出を実行する
   *
   * @param request 検出リクエスト
   * @param response 検出レスポンス
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
   *
   * resetTracking要求が来た場合は、
   * 新しいSquareDetectorへ作り直す。
   */
  std::unique_ptr<SquareDetector> detector;
};

#endif