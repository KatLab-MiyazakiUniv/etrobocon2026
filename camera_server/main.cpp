#include <iostream>
#include "Logger.h"
#include "SocketServer.h"
#include "RealNetworkSystem.h"
#include "FrameSave.h"

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

  // cv::Mat frame;
  // // フレームを取得
  // if(!camera.getFrame(frame)) {
  //   std::cerr << "フレームの取得に失敗しました" << std::endl;
  //   return -1;
  // }

  // // フレームを１枚保存
  // std::string filepath = "./frames";
  // std::string filename = "test_800_602";

  // // フレームを保存
  // FrameSave::save(frame, filepath, filename);

  ColorRegionDetectionActionHandler colorRegionDetectionHandler(camera);
  SocketServer server(colorRegionDetectionHandler, real);
  server.init();
  server.run();
  return 0;
}