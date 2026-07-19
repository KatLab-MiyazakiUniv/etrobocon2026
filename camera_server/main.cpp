#include <opencv2/imgcodecs.hpp>
#include "CameraCapture.h"
#include "Decode.h"
#include "FrameSave.h"
#include <opencv2/imgcodecs.hpp>
#include "CameraCapture.h"
#include "Decode.h"
#include "FrameSave.h"
#include "Logger.h"
#include "QrCodeDetector.h"

int main()
{
  Logger::init();

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
  SnapshotActionHandler snapshotHandler(camera);
  SocketServer server(snapshotHandler, colorRegionDetectionHandler, real);
  server.init();
  return 0;
}