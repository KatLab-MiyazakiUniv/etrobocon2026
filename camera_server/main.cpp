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

  // -----------------------------------------------------
  // 使用可能なカメラIDを探索
  // -----------------------------------------------------

  Logger::info(
      "CameraServer: カメラを探索します");

  const int cameraID =
      camera.findAvailableCameraID();

  if(cameraID < 0) {

    Logger::error(
        "CameraServer: 使用可能なカメラが見つかりません");

    return 1;
  }

  // -----------------------------------------------------
  // 使用するカメラIDを設定
  // -----------------------------------------------------

  if(!camera.setCameraID(cameraID)) {

    Logger::error(
        "CameraServer: カメラIDの設定に失敗しました");

    return 1;
  }

  Logger::printfLog(
      Logger::INFO,
      "CameraServer: カメラID %d を使用します",
      cameraID);

  // -----------------------------------------------------
  // カメラを開く
  // -----------------------------------------------------

  Logger::info(
      "CameraServer: カメラを開きます");

  if(!camera.openCamera()) {

    Logger::error(
        "CameraServer: カメラを開くことができませんでした");

    return 1;
  }

  Logger::info(
      "CameraServer: カメラの初期化に成功しました");

  // =====================================================
  // ActionHandler
  // =====================================================

  SnapshotActionHandler snapshotHandler(
      camera);

  ColorRegionDetectionActionHandler
      colorRegionDetectionHandler(
          camera);

  QrCodeDetectionActionHandler
      qrCodeDetectionHandler(
          camera);

  SquareDetectionActionHandler
      squareDetectionHandler(
          camera);

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
  // サーバー初期化
  // =====================================================

  if(!server.init()) {

    Logger::error(
        "CameraServer:初期化に失敗しました");

    return 1;
  }

  // =====================================================
  // サーバー起動
  // =====================================================

  Logger::info(
      "CameraServer: サーバーを起動します");

  server.run();

  return 0;
}