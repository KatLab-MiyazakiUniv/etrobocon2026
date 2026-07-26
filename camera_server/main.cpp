#include <iostream>
#include "Logger.h"
#include "SocketServer.h"
#include "RealNetworkSystem.h"
#include "FrameSave.h"
#include "QrCodeDetector.h"

int main()
{
  Logger::info("Hello KATLAB");
  RealNetworkSystem real;

  CameraCapture camera;
  int cameraId = camera.findAvailableCameraID();
  if(cameraId < 0) {
    Logger::error("利用可能なカメラを認識失敗");
    return -1;
  }
  camera.setCameraID(cameraId);
  if(!camera.openCamera()) {
    Logger::error("カメラの起動に失敗");
    return -1;
  }

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