/**
 * @file QrCodeDetectionActionHandlerTest.cpp
 * @brief QrCodeDetectionActionHandlerクラスのテスト
 * @author HaruArima08
 */

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include "CameraCapture.h"
#include "QrCodeDetectionActionHandler.h"
#include "Logger.h"
#include "MockCameraCapture.h"
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <string>

namespace etrobocon2026_test {

  // getFrameが失敗する場合のテスト
  TEST(QrCodeDetectionActionHandlerTest, ExecuteHandlesGetFrameFailure)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string fileName = "qr_handler_test_log.txt";
    std::string fullPath = logPath + fileName;
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
    Logger::init();
    Logger::setFileName(fileName, logPath);

    MockCameraCapture camera;
    camera.mockSuccess = false;
    QrCodeDetectionActionHandler handler(camera);
    CameraServer::QrCodeDetectorRequest request;
    request.roi = { 0, 0, 1920, 1080 };
    CameraServer::QrCodeDetectorResponse response;
    response.wasDetected = true;
    handler.execute(request, response);
    EXPECT_FALSE(response.wasDetected);

    Logger::outputToFile();
    std::ifstream file(fullPath);
    std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("QrCodeDetectionActionHandler:フレームの取得に失敗しました\n"),
              std::string::npos);
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
  }

  // getFrameが成功しQRコードが検出されるテスト
  TEST(QrCodeDetectionActionHandlerTest, ExecuteHandlesGetFrameSuccess)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string fileName = "qr_handler_test_log.txt";
    std::string fullPath = logPath + fileName;
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
    Logger::init();
    Logger::setFileName(fileName, logPath);

    MockCameraCapture camera;
    camera.mockSuccess = true;
    camera.mockFrame = cv::imread("camera_server/test_data/Hint1.png");
    ASSERT_FALSE(camera.mockFrame.empty());

    QrCodeDetectionActionHandler handler(camera);
    CameraServer::QrCodeDetectorRequest request;
    request.roi = { 0, 0, 1920, 1080 };
    CameraServer::QrCodeDetectorResponse response;
    response.wasDetected = false;

    handler.execute(request, response);

    EXPECT_TRUE(response.wasDetected);
    EXPECT_STREQ("25,35", response.content);

    Logger::outputToFile();
    std::ifstream file(fullPath);
    std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("QrCodeDetectionActionHandler:QRコードの検出に成功しました\n"),
              std::string::npos);
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
  }

  // getFrameが成功するがQRコードが検出されないテスト
  TEST(QrCodeDetectionActionHandlerTest, ExecuteHandlesQrCodeNotDetected)
  {
    const std::string logPath = "../tests/datafiles/logfiles/";
    const std::string fileName = "qr_handler_test_log.txt";
    std::string fullPath = logPath + fileName;
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
    Logger::init();
    Logger::setFileName(fileName, logPath);

    MockCameraCapture camera;
    camera.mockSuccess = true;
    camera.mockFrame = cv::Mat::zeros(1080, 1920, CV_8UC3);

    QrCodeDetectionActionHandler handler(camera);
    CameraServer::QrCodeDetectorRequest request;
    request.roi = { 0, 0, 1920, 1080 };
    CameraServer::QrCodeDetectorResponse response;
    response.wasDetected = true;

    handler.execute(request, response);

    EXPECT_FALSE(response.wasDetected);

    Logger::outputToFile();
    std::ifstream file(fullPath);
    std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("QrCodeDetectionActionHandler:QRコードが検出されませんでした\n"),
              std::string::npos);
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
  }

}  // namespace etrobocon2026_test
