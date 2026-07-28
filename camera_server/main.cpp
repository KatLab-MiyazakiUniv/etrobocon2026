/**
 * @file   main.cpp
 * @brief  QRコード常時検出処理
 */

#include <opencv2/imgcodecs.hpp>

#include "CameraCapture.h"
#include "Decode.h"
#include "FrameSave.h"
#include "Logger.h"
#include "QrCodeDetector.h"

int main()
{
  Logger::init();

  CameraCapture camera;

  int cameraId = camera.findAvailableCameraID();

  if(cameraId < 0) {
    Logger::error("No available camera found.");
    Logger::outputToFile();
    return 1;
  }

  camera.setCameraID(cameraId);

  if(!camera.openCamera()) {
    Logger::error("Failed to open camera.");
    Logger::outputToFile();
    return 1;
  }

  QrCodeDetector detector;

  cv::Mat frame;

  // 同じQRを何度も表示しないため
  std::string lastContent = "";

  while(true) {
    // カメラ画像取得
    if(!camera.getFrame(frame) || frame.empty()) {
      Logger::error("Failed to get frame.");

      continue;
    }

    // QR検出
    QrCodeDetectionResult result = detector.detect(frame);

    if(result.wasDetected) {
      // 新しいQRを検出した時だけ処理
      if(result.content != lastContent) {
        lastContent = result.content;

        Logger::printfLog(Logger::INFO, "QR detected: %s", result.content.c_str());

        // 暗号化QRの場合
        std::string decrypted = Decode::decrypt("1234", result.content);

        if(!decrypted.empty()) {
          Logger::printfLog(Logger::INFO, "Decrypted: %s", decrypted.c_str());
        }

        // 検出画像保存
        FrameSave::save(frame, "./frames", "detected_qr");
        // すべての処理が終わったら終了
        break;
      }
    }
  }

  Logger::outputToFile();

  return 0;
}