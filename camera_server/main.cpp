#include <iostream>
#include "Logger.h"
#include "SocketServer.h"
#include "RealNetworkSystem.h"

int main()
{
  Logger::info("Hello KATLAB");
  RealNetworkSystem real;

  CameraCapture camera;
  int cameraId = camera.findAvailableCameraID();
  if(cameraId < 0) {
    Logger::error("利用可能なカメラを認識失敗");
    return -1;
  }
  camera.setCameraID(cameraId);
  if(!camera.openCamera()) {
    Logger::error("カメラの起動に失敗");
    return -1;
  }

  ColorRegionDetectionActionHandler colorRegionDetectionHandler(camera);
  QrCodeDetectionActionHandler qrCodeDetectionHandler(camera);
  SnapshotActionHandler snapshotHandler(camera);
  SocketServer server(snapshotHandler, colorRegionDetectionHandler, qrCodeDetectionHandler, real);
  if(!server.init()) {
    Logger::error("サーバーの初期化に失敗");
    return -1;
  }
  server.run();
  return 0;
}
