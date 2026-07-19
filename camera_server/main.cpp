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
    return 1;
  }
  camera.setCameraID(cameraId);
  if(!camera.openCamera()) {
    Logger::error("Failed to open camera.");
    return 1;
  }

  cv::Mat frame;
  if(!camera.getFrame(frame) || frame.empty()) {
    Logger::error("Failed to get frame from camera.");
    Logger::outputToFile();
    return 1;
  }

  // ---- ヒントカード1（平文） ----
  {
    FrameSave::save(frame, "./frames", "hint_card_1");

    QrCodeDetector detector;
    QrCodeDetectionResult result = detector.detect(frame);

    Logger::printfLog(Logger::INFO, "ヒントカード1: wasDetected=%s",
                      result.wasDetected ? "true" : "false");
    if(result.wasDetected) {
      Logger::printfLog(Logger::INFO, "ヒントカード1: content=%s", result.content.c_str());
    } else {
      Logger::printfLog(Logger::ERROR, "ヒントカード1: QRコード検出失敗");
    }
  }

  // ---- ヒントカード2（暗号化） ----
  // {
  //   FrameSave::save(frame, "./frames", "hint_card_2");
  //
  //   QrCodeDetector detector;
  //   QrCodeDetectionResult result = detector.detect(frame);
  //
  //   Logger::printfLog(Logger::INFO, "ヒントカード2: wasDetected=%s",
  //                     result.wasDetected ? "true" : "false");
  //   if(result.wasDetected) {
  //     Logger::printfLog(Logger::INFO, "ヒントカード2: encrypted=%s", result.content.c_str());
  //     std::string decrypted = Decode::decrypt("1234", result.content);
  //     if(decrypted.empty()) {
  //       Logger::printfLog(Logger::ERROR, "ヒントカード2: 復号失敗");
  //     } else {
  //       Logger::printfLog(Logger::INFO, "ヒントカード2: decrypted=%s", decrypted.c_str());
  //     }
  //   }
  // }

  Logger::outputToFile();
  return 0;
}