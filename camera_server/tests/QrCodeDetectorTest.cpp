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

  // Hint1.pngの実解像度(135x135)を包含する、フレーム全体を対象とするROI
  static const cv::Rect FULL_FRAME_ROI(0, 0, 1920, 1080);

  // QRコードを含む画像を入力したときに、QRコードを検出し、
  // 内容「25,35」を正しく取得できることを確認するテスト
  TEST(QrCodeDetectorTest, DetectQrCode)
  {
    cv::Mat frame = cv::imread("camera_server/test_data/Hint1.png");

    ASSERT_FALSE(frame.empty());

    QrCodeDetector detector(FULL_FRAME_ROI);

    auto result = detector.detect(frame);

    EXPECT_TRUE(result.wasDetected);
    EXPECT_EQ("25,35", result.content);
  }

  // 空画像を入力したときに、QRコード未検出となり、
  // wasDetectedがfalse、contentが空文字列になることを確認するテスト
  TEST(QrCodeDetectorTest, EmptyFrame)
  {
    QrCodeDetector detector(FULL_FRAME_ROI);

    cv::Mat frame;

    QrCodeDetectionResult result = detector.detect(frame);

    EXPECT_FALSE(result.wasDetected);
    EXPECT_TRUE(result.content.empty());
  }

  // フレームは空でないが、QRコードを含まない画像を入力したときに、QRコード未検出となり、
  // wasDetectedがfalse、contentが空文字列になることを確認するテスト
  TEST(QrCodeDetectorTest, NoQrCode)
  {
    QrCodeDetector detector(FULL_FRAME_ROI);

    cv::Mat frame = cv::Mat::zeros(300, 300, CV_8UC3);

    QrCodeDetectionResult result = detector.detect(frame);

    EXPECT_FALSE(result.wasDetected);
    EXPECT_TRUE(result.content.empty());
  }

  // ROIがQRコードを含まない領域に限定されているとき、QRコード未検出となることを確認するテスト
  TEST(QrCodeDetectorTest, NotDetectedWhenRoiExcludesQrCode)
  {
    cv::Mat frame = cv::imread("camera_server/test_data/Hint1.png");
    ASSERT_FALSE(frame.empty());

    // フレーム左上の小さな領域にはQRコードが含まれない
    QrCodeDetector detector(cv::Rect(0, 0, 10, 10));

    auto result = detector.detect(frame);

    EXPECT_FALSE(result.wasDetected);
  }

  // ROIがフレーム範囲外を指定しているとき、QRコード未検出となることを確認するテスト
  TEST(QrCodeDetectorTest, NotDetectedWhenRoiOutsideFrame)
  {
    cv::Mat frame = cv::imread("camera_server/test_data/Hint1.png");
    ASSERT_FALSE(frame.empty());

    QrCodeDetector detector(cv::Rect(1000, 1000, 100, 100));

    auto result = detector.detect(frame);

    EXPECT_FALSE(result.wasDetected);
  }

  // setRoiでROIを更新すると、更新後のROIを用いて検出できることを確認するテスト
  TEST(QrCodeDetectorTest, DetectQrCodeAfterSetRoi)
  {
    cv::Mat frame = cv::imread("camera_server/test_data/Hint1.png");
    ASSERT_FALSE(frame.empty());

    // 最初はQRコードを含まない領域を指定
    QrCodeDetector detector(cv::Rect(0, 0, 10, 10));
    ASSERT_FALSE(detector.detect(frame).wasDetected);

    // フレーム全体を対象とするROIに更新
    detector.setRoi(FULL_FRAME_ROI);

    auto result = detector.detect(frame);
    EXPECT_TRUE(result.wasDetected);
    EXPECT_EQ("25,35", result.content);
  }
}  // namespace etrobocon2026_test