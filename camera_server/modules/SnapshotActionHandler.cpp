/**
 * @file SnapshotActionHandler.cpp
 * @brief スナップショット撮影をするクラス
 * @author sadomiya-sousi
 */

#include "SnapshotActionHandler.h"
#include "FrameSave.h"
#include <iostream>
#include "Logger.h"

SnapshotActionHandler::SnapshotActionHandler(CameraCapture& _camera) : camera(_camera) {}

void SnapshotActionHandler::execute(const CameraServer::SnapshotActionRequest& request,
                                    CameraServer::SnapshotActionResponse& response)
{
  Logger::printfLog(Logger::INFO, "Executing TAKE_SNAPSHOT command for file: %s", request.fileName);

  cv::Mat frame;
  if(!camera.getFrame(frame)) {
    Logger::error("Failed to capture frame for snapshot.");
    response.success = false;
    return;
  }
  FrameSave::save(frame, filePath, request.fileName);
  Logger::info("Snapshot saved successfully.");
  response.success = true;
}