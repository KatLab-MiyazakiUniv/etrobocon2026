/**
 * @file MultiThreadFrameSaveTest.cpp
 * @brief MultiThreadクラスを用いたFrameSave::saveの非同期実行テスト
 * @author sadomiya-sousi
 */

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "MultiThread.h"
#include "FrameSave.h"

namespace etrobocon2026_test {

  // FrameSave::save()を非同期で実行するテスト
  TEST(MultiThreadFrameSaveTest, WrapFrameSaveTest)
  {
    cv::Mat frame(10, 10, CV_8UC3, cv::Scalar(255, 255, 255));
    std::string filePath = "./test_output";
    std::string fileName = "test_async_frame";
    std::string fullPath = filePath + "/" + fileName + ".JPEG";

    // 事前に古いファイルが存在する場合は削除しておく
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }

    MultiThread::wrap([&]() { FrameSave::save(frame, filePath, fileName); });

    // ファイル生成を待機
    bool fileSaved = false;
    for(int i = 0; i < 100; ++i) {
      if(std::filesystem::exists(fullPath)) {
        fileSaved = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // ファイルが保存されたことを確認する
    EXPECT_TRUE(fileSaved);
    if(std::filesystem::exists(fullPath)) {
      std::filesystem::remove(fullPath);
    }
  }

}  // namespace etrobocon2026_test
