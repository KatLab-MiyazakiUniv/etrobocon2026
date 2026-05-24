/**
 * @file SocketServer.h
 * @brief 接続を待ち、クライアントからのリクエストを処理するクラス
 * @author okuyama0528, sadomiya-sousi
 */

#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H

#include "CameraCaptureHandler.h"
#include "ColorRegionDetecorHandler.h"
#include "BoundingBoxDetectorHandler.h"
#include <vector>

class SocketServer {
 public:
  /**
   * @brief コンストラクタ
   * @param CameraCaptureHandler カメラを制御するクラスアクションハンドラ
   * @param ColorRegionDetecorHandler 色領域検出をするアクションハンドラ
   * @param BoundingBoxDetectorHandler バウンディングボックスを作成するアクションハンドラ
   */
  SocketServer(CameraCaptureHandler& cameraCaptureHandler,
               ColorRegionDetecorHandler& colorRegionDetecorHandler,
               BoundingBoxDetectorHandler& boundingBoxDetectorHandler);

  /**
   * @brief サーバーを初期化する
   * @return true 初期化に成功した場合
   * @return false 初期化に失敗した場合
   */
  bool init();

  /**
   * @brief サーバーを実行し、クライアントからの接続を待機する
   */
  void run();

  /**
   * @brief サーバーをシャットダウンする
   */
  void shutdown();

 private:
  int listenSocket;
  bool isRunning;
  CameraCaptureHandler& cameraCaptureHandler;
  ColorRegionDetecorHandler& colorRegionDetecorHandler;
  BoundingBoxDetectorHandler& boundingBoxDetectorHandler;

  /**
   * @brief クライアントとの接続を処理する
   * @param clientSocket クライアントソケット
   */
  void handle_connection(int clientSocket);
};

#endif  // SOCKET_SERVER_H