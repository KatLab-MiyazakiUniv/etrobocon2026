/**
 * @file   QrCodeDetectorTest.cpp
 * @brief  QRコード検出用の画像処理クラスのテスト
 * @author yutaro-1214
 */

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "QrCodeDetector.h"

namespace etrobocon2026_test {

  // QRコードを正しく読み取れるかのテスト
  TEST(QrCodeDetectorTest, DetectQrCode)
  {
    cv::Mat frame = cv::imread("camera_server/test_data/Hint1.png");

    ASSERT_FALSE(frame.empty());

    QrCodeDetector detector;

    auto results = detector.detect(frame);

    EXPECT_TRUE(results[0].wasDetected);

    EXPECT_EQ("25,35", results[0].content);
  }

  // 空フレームを渡したときのテスト
  TEST(QrCodeDetectorTest, EmptyFrame)
  {
    QrCodeDetector detector;

    cv::Mat frame;

    auto results = detector.detect(frame);

    EXPECT_TRUE(results.empty());
  }

  // QRコードがないフレームを渡したときのテスト
  TEST(QrCodeDetectorTest, NoQrCode)
  {
    QrCodeDetector detector;

    cv::Mat frame = cv::Mat::zeros(300, 300, CV_8UC3);

    auto results = detector.detect(frame);

    EXPECT_TRUE(results.empty());
  }

  // 複数のQRコードを正しく読み取れるかのテスト
  TEST(QrCodeDetectorTest, DetectMultipleQrCodes)
  {
    cv::Mat frame = cv::imread("camera_server/test_data/Multiple.png");

    ASSERT_FALSE(frame.empty());

    QrCodeDetector detector;

    auto results = detector.detect(frame);

    for(const auto& result : results) {
      EXPECT_TRUE(result.wasDetected);
      EXPECT_FALSE(result.content.empty());
    }
  }
}  // namespace etrobocon2026_test
