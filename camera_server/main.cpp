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
    Logger::error("No available camera found.");
    return 1;
  }
  camera.setCameraID(cameraId);
  if(!camera.openCamera()) {
    Logger::error("Failed to open camera.");
    return 1;
  }

  ColorRegionDetectionActionHandler colorRegionDetectionHandler(camera);
  SnapshotActionHandler snapshotActionHandler(camera);

  SocketServer server(colorRegionDetectionHandler, snapshotActionHandler, real);
  server.init();
  server.run();
  return 0;
}