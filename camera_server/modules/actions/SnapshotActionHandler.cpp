/**
 * @file SnapshotActionHandler.cpp
 * @brief スナップショット撮影をするクラス
 * @author sadomiya-sousi
 */

#include "SnapshotActionHandler.h"

SnapshotActionHandler::SnapshotActionHandler(CameraCapture& _camera) : camera(_camera) {}

void SnapshotActionHandler::execute(const CameraServer::SnapshotActionRequest& request,
                                    CameraServer::SnapshotActionResponse& response)
{
  Logger::printfLog(Logger::INFO, "SnapshotActionHandler:実行: %s", request.fileName);

  cv::Mat frame;
  if(!camera.getFrame(frame)) {
    Logger::error("SnapshotActionHandler:フレーム取得失敗");
    response.success = false;
    return;
  }
  FrameSave::save(frame, filePath, request.fileName);
  Logger::info("SnapshotActionHandler:保存成功");
  response.success = true;
}
