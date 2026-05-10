
/**
 * @file   CameraCaptureTest.cpp
 * @brief  色領域検出用の画像処理クラスのテスト
 * @author okuyama0528 sadomiya
 */

#include <gtest/gtest.h>
#include "ColorRegionDetector.h"
#include "FrameSave.h"
#include <opencv2/opencv.hpp>

namespace etrobocon2026_test {
  TEST(ColorRegionDetectorTest, detect)
  {
    std::vector<cv::Scalar> lowers = { cv::Scalar(0, 0, 0) };

    std::vector<cv::Scalar> uppers = { cv::Scalar(180, 255, 80) };

    cv::Rect roi(50, 240, 540, 240);
    cv::Size resolution(640, 480);

    ColorRegionDetector detector(lowers, uppers, roi, resolution);

    // 画像読み込み
    cv::Mat frame = cv::imread("../camera_server/tests/dummy_frame.JPEG");

    // テストとしてちゃんと失敗させる
    ASSERT_FALSE(frame.empty()) << "画像の読み込みに失敗";

    BoundingBoxDetectionResult result;
    detector.detect(frame, result);

    // テスト判定
    EXPECT_TRUE(result.wasDetected);

    // 描画
    if(result.wasDetected) {
      cv::rectangle(frame, result.topLeft, result.bottomRight, cv::Scalar(0, 255, 0), 2);
    }

    // 保存
    std::string filepath = "./frames";
    std::string filename = "result";

    FrameSave::save(frame, filepath, filename);

    std::cout << "Saved frame successfully." << std::endl;
  }

}  // namespace etrobocon2026_test
