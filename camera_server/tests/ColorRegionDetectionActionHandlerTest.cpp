/**
 * @file ColorRegionDetectionActionHandlerTest.cpp
 * @brief ColorRegionDetectionActionHandlerクラスのテスト
 * @author sadomiya-sousi
 */

#include <gtest/gtest.h>
#include "CameraCapture.h"
#include "ColorRegionDetectionActionHandler.h"
#include "ImageProcessingColor.h"
#include <cstdio>
#include "Logger.h"
#include <fstream>
#include <filesystem>
#include <string>
#include "MockCameraCapture.h"

namespace etrobocon2026_test {

  // getFrameが失敗する場合のテスト
  TEST(ColorRegionDetectionActionHandlerTest, ExecuteHandlesGetFrameFailure)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string fileName = "handler_test_log.txt";
    std::string fullPath = logPath + fileName;
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
    Logger::init();
    Logger::setFileName(fileName, logPath);

    MockCameraCapture camera;
    camera.mockSuccess = false;
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

    Logger::outputToFile();
    std::ifstream file(fullPath);
    std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("ColorRegionDetectionActionHandler:フレームの取得に失敗しました\n"),
              std::string::npos);
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
  }

  // getFrameが成功し色領域が検出されるテスト
  TEST(ColorRegionDetectionActionHandlerTest, ExecuteHandlesGetFrameSuccess)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string fileName = "handler_test_log.txt";
    std::string fullPath = logPath + fileName;
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
    Logger::init();
    Logger::setFileName(fileName, logPath);

    MockCameraCapture camera;
    camera.mockSuccess = true;
    camera.mockFrame = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::rectangle(camera.mockFrame, cv::Rect(200, 200, 100, 100), cv::Scalar(0, 0, 0), cv::FILLED);

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

    Logger::outputToFile();
    std::ifstream file(fullPath);
    std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("ColorRegionDetectionActionHandler:色領域の検出に成功しました\n"),
              std::string::npos);
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
  }

}  // namespace etrobocon2026_test