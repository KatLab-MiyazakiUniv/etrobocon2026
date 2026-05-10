/**
 * @file   CameraCaptureTest.cpp
 * @brief  CameraCaptureクラスのテスト
 * @author HaruArima08
 */

#include <gtest/gtest.h>
#include "CameraCapture.h"
#include "ColorRegionDetector.h"

namespace etrobocon2026_test {

  TEST(ColorRegionDetectorTest, detect)
  {
    std::vector<cv::Scalar> lowers
        = { cv::Scalar(0, 100, 100), cv::Scalar(50, 100, 100), cv::Scalar(100, 100, 100) };

    std::vector<cv::Scalar> uppers
        = { cv::Scalar(10, 255, 255), cv::Scalar(70, 255, 255), cv::Scalar(130, 255, 255) };

    cv::Rect roi(50, 240, 540, 240);
    cv::Size resolution(640, 480);

    ColorRegionDetector detector(lowers, uppers, roi, resolution);

    // 画像の読み込み
    cv::Mat img = cv::imread("C:\Users\okuyamakota\RasPike-ART\sdk\workspace\etrobocon2026\camera_"
                             "server\tests\dummy_frame.JPEG");

    // 読み込みチェック
    if(img.empty()) {
      std::cerr << "画像の読み込みに失敗しました" << std::endl;
      return -1;
    }

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
  }
}  // namespace etrobocon2026_test