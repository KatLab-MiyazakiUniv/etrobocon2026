/**
 * @file QrCodeDetectionActionHandler.cpp
 * @brief QRコードを検出するクラス
 * @author HaruArima08
 */

#include "QrCodeDetectionActionHandler.h"

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
  cv::Mat frame;
  if(!camera.getFrame(frame)) {
    Logger::error("QrCodeDetectionActionHandler:フレームの取得に失敗しました");
    response.wasDetected = false;
    return;
  }

  cv::Rect localRoi(request.roi.x, request.roi.y, request.roi.width, request.roi.height);
  detector.setRoi(localRoi);

  QrCodeDetectionResult result = detector.detect(frame);

  response.wasDetected = result.wasDetected;
  if(result.wasDetected) {
    for(uint32_t i = 0; i < CameraServer::QR_CODE_CORNER_COUNT; i++) {
      response.corners[i].x = static_cast<int32_t>(std::lround(result.corners[i].x));
      response.corners[i].y = static_cast<int32_t>(std::lround(result.corners[i].y));
    }

    Logger::info("QrCodeDetectionActionHandler:QRコードの検出に成功しました");
  } else {
    Logger::error("QrCodeDetectionActionHandler:QRコードが検出されませんでした");
  }

  std::string directoryPath = "datafiles/line_trace";

  cv::Mat saveFrame = frame.clone();
  QrCodeDetectionResult saveResult = result;
  cv::Rect saveRoi = localRoi;

  MultiThread::wrap([saveFrame, directoryPath, saveResult, saveRoi]() mutable {
    FrameSave::save(saveFrame, directoryPath, saveResult, saveRoi);
  });
}

const CameraCapture& QrCodeDetectionActionHandler::getCamera() const
{
  return camera;
}

const QrCodeDetector& QrCodeDetectionActionHandler::getDetector() const
{
  return detector;
}
