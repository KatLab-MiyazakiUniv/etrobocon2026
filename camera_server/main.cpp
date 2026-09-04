/**
 * @file main.cpp
 * @brief カメラサーバーのエントリーポイント
 */

#include "CameraCapture.h"
#include "ColorRegionDetectionActionHandler.h"
#include "Logger.h"
#include "QrCodeDetectionActionHandler.h"
#include "RealNetworkSystem.h"
#include "SnapshotActionHandler.h"
#include "SocketServer.h"
#include "SquareDetectionActionHandler.h"

int main()
{
  Logger::info("Hello KATLAB");

  // =====================================================
  // ネットワーク
  // =====================================================

  RealNetworkSystem real;

  // =====================================================
  // カメラ
  // =====================================================

  CameraCapture camera;

  // =====================================================
  // ActionHandler
  // =====================================================

  SnapshotActionHandler snapshotHandler(camera);

  ColorRegionDetectionActionHandler colorRegionDetectionHandler(camera);

  QrCodeDetectionActionHandler qrCodeDetectionHandler(camera);

  /*
   * 正方形検出用ハンドラ
   */
  SquareDetectionActionHandler squareDetectionHandler(camera);

  // =====================================================
  // SocketServer
  // =====================================================

  SocketServer server(
      snapshotHandler,
      colorRegionDetectionHandler,
      qrCodeDetectionHandler,
      squareDetectionHandler,
      real);

  // =====================================================
  // サーバー起動
  // =====================================================

  if(!server.init()) {
    Logger::error("CameraServer:初期化に失敗しました");
    return 1;
  }

  server.run();

  return 0;
}