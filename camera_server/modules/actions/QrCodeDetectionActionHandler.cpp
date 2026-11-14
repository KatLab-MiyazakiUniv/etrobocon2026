/**
 * @file QrCodeDetectionActionHandler.cpp
 * @brief QRコードを検出するクラス
 * @author HaruArima08
 */

#include "QrCodeDetectionActionHandler.h"

#include <functional>
#include "MultiThread.h"
#include "./../utils/FrameSave.h"
// #include "FrameSave.h"


QrCodeDetectionActionHandler::QrCodeDetectionActionHandler(CameraCapture& _camera)
  : camera(_camera), detector(cv::Rect(0, 0, CAM_MAX_WIDTH, CAM_MAX_HEIGHT))
{
  LOG_CREATE("QrCodeDetectionActionHandler");
}

QrCodeDetectionActionHandler::~QrCodeDetectionActionHandler()
{
  LOG_DESTROY("QrCodeDetectionActionHandler");
}

void QrCodeDetectionActionHandler::execute(const CameraServer::QrCodeDetectorRequest& request,
                                           CameraServer::QrCodeDetectorResponse& response)
{
  response = {};

  cv::Mat frame;
  if(!camera.getFrame(frame)) {
    Logger::error("QrCodeDetectionActionHandler:フレームの取得に失敗しました");
    response.wasDetected = false;
    return;
  }



  cv::Rect localRoi(request.roi.x, request.roi.y, request.roi.width, request.roi.height);
  detector.setValidatedRoi(localRoi);

  QrCodeDetectionResult result = detector.detect(frame);

  response.wasDetected = result.wasDetected;
  if(result.wasDetected) {
    // 検出したQRコードの文字列をレスポンスに格納する
    std::strncpy(response.content, result.content.c_str(), sizeof(response.content) - 1);

    // 検出したQRコードの4頂点の座標をレスポンスに格納する
    for(uint32_t i = 0; i < CameraServer::QR_CODE_CORNER_COUNT; i++) {
      response.corners[i].x = static_cast<int32_t>(std::lround(result.corners[i].x));
      response.corners[i].y = static_cast<int32_t>(std::lround(result.corners[i].y));
    }

    Logger::info("QrCodeDetectionActionHandler:QRコードの検出に成功しました");
  } else {
    Logger::error("QrCodeDetectionActionHandler:QRコードが検出されませんでした");
  }


//   std::string directoryPath = "datafiles/line_trace";


// // clang-format off
//   std::string fileName = "det_d" + std::to_string(1) +
//                           "_tlx" + Utils::toStrInt(0) +
//                           "_tly" + Utils::toStrInt(0) +
//                           "_trx" + Utils::toStrInt(0) +
//                           "_try" + Utils::toStrInt(0) +
//                           "_blx" + Utils::toStrInt(0) +
//                           "_bly" + Utils::toStrInt(0) +
//                           "_brx" + Utils::toStrInt(0) +
//                           "_bry" + Utils::toStrInt(0) +
//                           "_rx"  + Utils::toStrInt(0) +
//                           "_ry"  + Utils::toStrInt(0) +
//                           "_rw"  + Utils::toStrInt(0) +
//                           "_rh"  + Utils::toStrInt(0) +
//                           "_qrd" + std::to_string(0) +
//                           "_"    + std::to_string(ClockUtil::now());
//   // clang-format on



//   MultiThread::wrap(
//       [=]() mutable { FrameSave::save(frame, directoryPath,fileName); });
//   int totalEndTime = ClockUtil::now();

}
