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

  SocketServer server(colorRegionDetectionHandler, real);
  server.init();
  server.run();
  return 0;
}