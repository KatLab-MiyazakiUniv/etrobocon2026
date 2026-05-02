#include <iostream>
#include "CameraCapture.h"
#include "FrameSave.h"
#include <string>
#include <vector>

int main()
{
  std::cout << "Hello KATLABBBBBB" << std::endl;

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

  // フレームを取得
  cv::Mat frame;
  if(!camera.getFrame(frame)) {
    std::cerr << "フレームの取得に失敗しました" << std::endl;
    return -1;
  }

  // フレームを１枚保存
  std::string filepath = "./frames";
  std::string filename = "test_1920_1080";

  // フレームを保存
  FrameSave::save(frame, filepath, filename);

  // 幅と高さを設定
  camera.setCapProps(800, 600);

  // フレームを取得
  if(!camera.getFrame(frame)) {
    std::cerr << "フレームの取得に失敗しました" << std::endl;
    return -1;
  }

  // フレームを１枚保存
  filepath = "./frames";
  filename = "test_800_600";

  // フレームを保存
  FrameSave::save(frame, filepath, filename);

  camera.setCapProps(320, 240);

  // フレームを取得
  if(!camera.getFrame(frame)) {
    std::cerr << "フレームの取得に失敗しました" << std::endl;
    return -1;
  }

  // フレームを１枚保存
  filepath = "./frames";
  filename = "test_320_240";

  // フレームを保存
  FrameSave::save(frame, filepath, filename);

  camera.setCapProps(1920, 1080);

  // フレームを１0枚保存
  std::vector<cv::Mat> frames;
  if(!camera.getFrames(frames, 10, 10)) {
    std::cerr << "フレームの取得に失敗しました" << std::endl;
    return -1;
  }

  std::cout << frames.size() << std::endl;
  int frameslength = frames.size();

  int i = 0;
  for(cv::Mat f : frames) {
    filename = "test_" + std::to_string(i);
    FrameSave::save(f, filepath, filename);
    i++;
  }
  std::cerr << "フレームを" << frameslength << "枚保存しました" << std::endl;

  return 0;
}