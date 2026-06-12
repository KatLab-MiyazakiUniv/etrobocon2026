/**
 * @file ColorRegionDetectionActionHandlerTest.cpp
 * @brief ColorRegionDetectionActionHandlerクラスのテスト
 * @author sadomiya-sousi
 */

#include <gtest/gtest.h>
#define private public  // メンバ変数を変更したいので,アクセス指定子をprivateからpublicに変更
#include "CameraCapture.h"
#include "ColorRegionDetectionActionHandler.h"
#undef private
#include "ImageProcessingColor.h"
#include <cstdio>

namespace etrobocon2026_test {

  // getFrameが失敗する場合のテスト
  TEST(ColorRegionDetectionActionHandlerTest, ExecuteHandlesGetFrameFailure)
  {
    CameraCapture camera;
    ColorRegionDetectionActionHandler handler(camera);
    CameraServer::ColorRegionDetectorRequest request;
    request.hsvRangeCount = 1;
    request.hsvRanges[0] = ImageProcessingColor::getHSVRangeFromColor(ImageProcessingColor::BLACK);
    request.roi = { 0, 0, 640, 480 };
    request.requireLargestColorIndex = false;
    CameraServer::ColorRegionDetectorResponse response;
    response.result.wasDetected = true;
    handler.execute(request, response);
    EXPECT_FALSE(response.result.wasDetected);
  }

  // getFrameが成功し色領域が検出されるテスト,
  TEST(ColorRegionDetectionActionHandlerTest, ExecuteHandlesGetFrameSuccess)
  {
    cv::Mat dummyFrame(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::rectangle(dummyFrame, cv::Rect(200, 200, 100, 100), cv::Scalar(0, 0, 0), cv::FILLED);

    const char* dummyPath = "dummy_frame_success.png";
    cv::imwrite(dummyPath, dummyFrame);

    CameraCapture camera;
    // cap.openで取得するフレームを事前に代入
    camera.cap.open(dummyPath);
    ASSERT_TRUE(camera.cap.isOpened());
    ColorRegionDetectionActionHandler handler(camera);
    CameraServer::ColorRegionDetectorRequest request;
    request.hsvRangeCount = 1;
    // 黒色を検出するHSV範囲
    request.hsvRanges[0] = ImageProcessingColor::getHSVRangeFromColor(ImageProcessingColor::BLACK);
    request.roi = { 50, 50, 540, 380 };
    request.requireLargestColorIndex = false;

    CameraServer::ColorRegionDetectorResponse response;
    response.result.wasDetected = false;

    handler.execute(request, response);

    EXPECT_TRUE(response.result.wasDetected);
    EXPECT_EQ(response.result.topLeft.x, 200);
    EXPECT_EQ(response.result.topLeft.y, 200);
    EXPECT_EQ(response.result.bottomRight.x, 300);
    EXPECT_EQ(response.result.bottomRight.y, 300);

    std::remove(dummyPath);
  }

  // ColorRegionDetector の再生成がスキップされるか検証
  TEST(ColorRegionDetectionActionHandlerTest, ExecuteSkipsDetectorRecreation)
  {
    cv::Mat dummyFrame(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::rectangle(dummyFrame, cv::Rect(200, 200, 100, 100), cv::Scalar(0, 0, 0), cv::FILLED);
    const char* dummyPath = "dummy_frame_skip.png";
    cv::imwrite(dummyPath, dummyFrame);
    CameraCapture camera;
    camera.cap.open(dummyPath);
    ASSERT_TRUE(camera.cap.isOpened());
    ColorRegionDetectionActionHandler handler(camera);
    CameraServer::ColorRegionDetectorRequest request;
    request.hsvRangeCount = 1;
    request.hsvRanges[0] = ImageProcessingColor::getHSVRangeFromColor(ImageProcessingColor::BLACK);
    request.roi = { 50, 50, 540, 380 };
    request.requireLargestColorIndex = false;

    CameraServer::ColorRegionDetectorResponse response;
    response.result.wasDetected = false;
    handler.execute(request, response);
    ASSERT_TRUE(response.result.wasDetected);
    // 生成された detector のポインタアドレスを取得
    ColorRegionDetector* firstDetectorPtr = handler.detector.get();
    ASSERT_NE(firstDetectorPtr, nullptr);
    camera.cap.open(dummyPath);
    handler.execute(request, response);
    ASSERT_TRUE(response.result.wasDetected);

    // 2回目の実行後のポインタアドレスを取得し、1回目と同じ（インスタンスが再利用された）ことを検証
    ColorRegionDetector* secondDetectorPtr = handler.detector.get();
    EXPECT_EQ(firstDetectorPtr, secondDetectorPtr);
    std::remove(dummyPath);
  }

}  // namespace etrobocon2026_test
