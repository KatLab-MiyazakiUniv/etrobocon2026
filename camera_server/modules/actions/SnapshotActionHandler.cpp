/**
 * @file SnapshotActionHandler.cpp
 * @brief スナップショット撮影をするクラス
 * @author sadomiya-sousi
 */

#include "SnapshotActionHandler.h"
#include "FrameSave.h"
#include <iostream>

SnapshotActionHandler::SnapshotActionHandler(CameraCapture& _camera) : camera(_camera) {}

void SnapshotActionHandler::execute(const CameraServer::SnapshotActionRequest& request,
                                    CameraServer::SnapshotActionResponse& response)
{
  std::cout << "Executing TAKE_SNAPSHOT command for file: " << request.fileName << std::endl;

  cv::Mat frame;
  if(!camera.getFrame(frame)) {
    std::cerr << "Failed to capture frame for snapshot." << std::endl;
    response.success = false;
    return;
  }
  FrameSave::save(frame, filePath, request.fileName);
  std::cout << "Snapshot saved successfully." << std::endl;
  response.success = true;
}
