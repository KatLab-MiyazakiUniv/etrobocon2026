/**
 * @file   ColorRegionDetectorTest.cpp
 * @brief  色領域検出用の画像処理クラスのテスト
 * @author HaruArima08, okuyama0528, sadomiya-sousi
 */

#include <gtest/gtest.h>
#include "ColorRegionDetector.h"

namespace etrobocon2026_test {

  class ColorRegionDetectorTest : public ::testing::Test {
   protected:
    // 黒色を検出する設定
    std::vector<HSVRange> blackRanges = { { cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 50) } };
    cv::Rect defaultROI = cv::Rect(50, 240, 540, 240);
    cv::Size defaultRes = cv::Size(640, 480);

    // 白フレームの指定領域に黒色矩形を描画した合成フレームを返す
    cv::Mat makeFrameWithBlackRect(cv::Size frameSize, cv::Rect rect)
    {
      cv::Mat frame(frameSize, CV_8UC3, cv::Scalar(255, 255, 255));  // 白背景
      cv::rectangle(frame, rect, cv::Scalar(0, 0, 0), cv::FILLED);   // 黒色矩形
      return frame;
    }
  };

  // 空フレームを渡したとき wasDetected == false になるかのテスト
  TEST_F(ColorRegionDetectorTest, EmptyFrameNotDetected)
  {
    // ColorRegionDetector detector(blackRanges, defaultROI, defaultRes);
    ColorRegionDetector detector(blackRanges, defaultROI);
    BoundingBoxDetectionResult result;
    detector.detect(cv::Mat(), result);
    EXPECT_FALSE(result.wasDetected);
  }

  // 対象色が存在しない白フレームのとき wasDetected == false になるかのテスト
  TEST_F(ColorRegionDetectorTest, NoTargetColorNotDetected)
  {
    // ColorRegionDetector detector(blackRanges, defaultROI, defaultRes);
    ColorRegionDetector detector(blackRanges, defaultROI);
    cv::Mat frame(defaultRes, CV_8UC3, cv::Scalar(255, 255, 255));
    BoundingBoxDetectionResult result;
    detector.detect(frame, result);
    EXPECT_FALSE(result.wasDetected);
  }

  // ROI内に十分な面積の対象色の色領域があるとき wasDetected == true になるかのテスト
  TEST_F(ColorRegionDetectorTest, ColorInROIDetected)
  {
    // ColorRegionDetector detector(blackRanges, defaultROI, defaultRes);
    ColorRegionDetector detector(blackRanges, defaultROI);
    // defaultROI = (50, 240, 540, 240) の内側に黒色矩形を配置
    cv::Mat frame = makeFrameWithBlackRect(defaultRes, cv::Rect(200, 300, 200, 100));
    BoundingBoxDetectionResult result;
    detector.detect(frame, result);
    EXPECT_TRUE(result.wasDetected);
  }

  // ROI外に対象色の色領域があるとき wasDetected == false になるかのテスト
  TEST_F(ColorRegionDetectorTest, ColorOutsideROINotDetected)
  {
    // ColorRegionDetector detector(blackRanges, defaultROI, defaultRes);
    ColorRegionDetector detector(blackRanges, defaultROI);
    // ROI の y 開始 240 より上（y=50）に黒色矩形を配置
    cv::Mat frame = makeFrameWithBlackRect(defaultRes, cv::Rect(200, 50, 200, 100));
    BoundingBoxDetectionResult result;
    detector.detect(frame, result);
    EXPECT_FALSE(result.wasDetected);
  }

  // MIN_CONTOUR_AREA(=50) 未満の色領域は wasDetected == false になるかのテスト
  TEST_F(ColorRegionDetectorTest, TinyColorAreaNotDetected)
  {
    // ColorRegionDetector detector(blackRanges, defaultROI, defaultRes);
    ColorRegionDetector detector(blackRanges, defaultROI);
    // 5×5=25px はモルフォロジー後も面積閾値50を下回る
    cv::Mat frame = makeFrameWithBlackRect(defaultRes, cv::Rect(200, 300, 5, 5));
    BoundingBoxDetectionResult result;
    detector.detect(frame, result);
    EXPECT_FALSE(result.wasDetected);
  }

  // バウンディングボックスの座標がフレーム全体基準（ROIオフセット加算済み）の値で返るかのテスト
  TEST_F(ColorRegionDetectorTest, BoundingBoxCoordinatesIncludeROIOffset)
  {
    // ColorRegionDetector detector(blackRanges, defaultROI, defaultRes);
    ColorRegionDetector detector(blackRanges, defaultROI);
    cv::Rect targetRect(200, 300, 100, 80);
    cv::Mat frame = makeFrameWithBlackRect(defaultRes, targetRect);
    BoundingBoxDetectionResult result;
    detector.detect(frame, result);

    ASSERT_TRUE(result.wasDetected);
    EXPECT_EQ(result.topLeft.x, targetRect.x);
    EXPECT_EQ(result.topLeft.y, targetRect.y);
    EXPECT_EQ(result.topRight.x, targetRect.x + targetRect.width);
    EXPECT_EQ(result.topRight.y, targetRect.y);
    EXPECT_EQ(result.bottomLeft.x, targetRect.x);
    EXPECT_EQ(result.bottomLeft.y, targetRect.y + targetRect.height);
    EXPECT_EQ(result.bottomRight.x, targetRect.x + targetRect.width);
    EXPECT_EQ(result.bottomRight.y, targetRect.y + targetRect.height);
  }

  // 複数の色領域があるとき、最大面積の輪郭に対するバウンディングボックスが返るかのテスト
  TEST_F(ColorRegionDetectorTest, LargestContourIsSelected)
  {
    // ColorRegionDetector detector(blackRanges, defaultROI, defaultRes);
    ColorRegionDetector detector(blackRanges, defaultROI);
    cv::Mat frame(defaultRes, CV_8UC3, cv::Scalar(255, 255, 255));

    cv::Rect largeRect(100, 280, 200, 100);
    cv::Rect smallRect(450, 350, 20, 20);
    cv::rectangle(frame, largeRect, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(frame, smallRect, cv::Scalar(0, 0, 0), cv::FILLED);

    BoundingBoxDetectionResult result;
    detector.detect(frame, result);

    ASSERT_TRUE(result.wasDetected);
    // バウンディングボックスが大きい方の矩形の範囲に対応している
    EXPECT_EQ(result.topLeft.x, largeRect.x);
    EXPECT_EQ(result.topLeft.y, largeRect.y);
    EXPECT_EQ(result.bottomRight.x, largeRect.x + largeRect.width);
    EXPECT_EQ(result.bottomRight.y, largeRect.y + largeRect.height);
  }

  // 複数色を指定し、隣接する色領域が一つのバウンディングボックスになるかのテスト
  TEST_F(ColorRegionDetectorTest, MultipleColorRangesAreORed)
  {
    // 黒色と青色を検出する設定
    std::vector<HSVRange> ranges = blackRanges;
    ranges.push_back({ cv::Scalar(110, 200, 200), cv::Scalar(130, 255, 255) });

    // ColorRegionDetector detector(ranges, defaultROI, defaultRes);
    ColorRegionDetector detector(ranges, defaultROI);

    cv::Mat frame(defaultRes, CV_8UC3, cv::Scalar(255, 255, 255));

    // 黒色矩形と青色矩形を隣接配置
    cv::Rect blackRect(100, 280, 150, 80);
    cv::Rect blueRect(250, 280, 100, 80);
    cv::rectangle(frame, blackRect, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(frame, blueRect, cv::Scalar(255, 0, 0), cv::FILLED);

    BoundingBoxDetectionResult result;
    detector.detect(frame, result);

    ASSERT_TRUE(result.wasDetected);
    // バウンディングボックスが両矩形を合わせた領域を囲んでいる
    EXPECT_EQ(result.topLeft.x, blackRect.x);
    EXPECT_EQ(result.topLeft.y, blackRect.y);
    EXPECT_EQ(result.bottomRight.x, blueRect.x + blueRect.width);
    EXPECT_EQ(result.bottomRight.y, blueRect.y + blueRect.height);
  }
}  // namespace etrobocon2026_test
