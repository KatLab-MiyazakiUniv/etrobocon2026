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

    auto result = detector.detect(frame);

    EXPECT_TRUE(result.wasDetected);
    EXPECT_EQ("25,35", result.content);
  }

  // 空画像を渡したときのテスト
  TEST(QrCodeDetectorTest, EmptyFrame)
  {
    QrCodeDetector detector;

    cv::Mat frame;

    QrCodeDetectionResult result = detector.detect(frame);

    EXPECT_FALSE(result.wasDetected);
    EXPECT_TRUE(result.content.empty());
  }

  // QRコードがない画像を渡したときのテスト
  TEST(QrCodeDetectorTest, NoQrCode)
  {
    QrCodeDetector detector;

    cv::Mat frame = cv::Mat::zeros(300, 300, CV_8UC3);

    QrCodeDetectionResult result = detector.detect(frame);

    EXPECT_FALSE(result.wasDetected);
    EXPECT_TRUE(result.content.empty());
  }
}  // namespace etrobocon2026_test