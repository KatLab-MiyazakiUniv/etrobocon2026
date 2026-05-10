/*#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "CameraCapture.h"
#include "FrameSave.h"

int main()
{
  std::cout << "Hello KATLABBBBBB" << std::endl;

  // カメラ初期化
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

  // フレーム取得（安定化のためリトライ）
  cv::Mat frame;
  bool ok = false;

  for(int i = 0; i < 10; i++) {
    if(camera.getFrame(frame)) {
      ok = true;
      break;
    }
  }

  if(!ok) {
    std::cerr << "フレームの取得に失敗しました" << std::endl;
    return -1;
  }
  // ここで画像処理
  ColorRegionDetector detector;
  cv::Scalar lowerHSV(0, 100, 100);
  cv::Scalar upperHSV(10, 255, 255);

  detector.setRange(lowerHSV, upperHSV);
  cv::Mat result = detector.detect(frame);

    // 保存
  std::string filepath = "./frames";
  std::string filename = "test_1920_1080";

  FrameSave::save(frame, filepath, filename);

  std::cout << "Saved frame successfully." << std::endl;

  return 0;
}
*/
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "CameraCapture.h"
#include "FrameSave.h"
#include "ColorRegionDetector.h"

int main()
{
  std::cout << "Hello KATLABBBBBB" << std::endl;

  // カメラ初期化
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

  // フレーム取得
  cv::Mat frame;
  bool ok = false;

  for(int i = 0; i < 10; i++) {
    if(camera.getFrame(frame)) {
      ok = true;
      break;
    }
  }

  if(!ok) {
    std::cerr << "フレームの取得に失敗しました" << std::endl;
    return -1;
  }

  // =========================
  // ★ここが変更ポイント
  // =========================

  std::vector<cv::Scalar> lowers
      = { cv::Scalar(0, 100, 100), cv::Scalar(50, 100, 100), cv::Scalar(100, 100, 100) };

  std::vector<cv::Scalar> uppers
      = { cv::Scalar(10, 255, 255), cv::Scalar(70, 255, 255), cv::Scalar(130, 255, 255) };

  cv::Rect roi(50, 240, 540, 240);
  cv::Size resolution(640, 480);

  ColorRegionDetector detector(lowers, uppers, roi, resolution);

  BoundingBoxDetectionResult result;
  detector.detect(frame, result);

  if(result.wasDetected) {
    std::cout << "Detected!" << std::endl;
  } else {
    std::cout << "Not detected" << std::endl;
  }

  // 保存（元画像）
  std::string filepath = "./frames";
  std::string filename = "test_1920_1080";

  FrameSave::save(frame, filepath, filename);

  std::cout << "Saved frame successfully." << std::endl;

  return 0;
}