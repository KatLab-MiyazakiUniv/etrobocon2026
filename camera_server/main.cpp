#include <iostream>
#include "CameraCapture.h"

int main()
{
  std::cout << "Camera server starting..." << std::endl;

  // カメラの初期化
  CameraCapture camera;
  int cameraId = camera.findAvailableCameraID();
  if(cameraId < 0) {
    std::cerr << "No available camera found." << std::endl;
    return 1;
  }
  camera.setCameraID(cameraId);
  if(!camera.openCamera()) {
    std::cerr << "Failed to open camera." << std::endl;
    return 1;
  }
  std::cout << "Camera initialized successfully." << std::endl;

  return 0;
}