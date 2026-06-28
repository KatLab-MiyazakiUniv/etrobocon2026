/**
 * @file   Snapshot.cpp
 * @brief  サーバーに写真撮影を依頼するクラス
 * @author sadomiya-sousi
 */

#include "Snapshot.h"

#include "Logger.h"
#include "SocketClient.h"
#include <cstring>

Snapshot::Snapshot(Robot& _robot, const std::string& _fileName,
                   std::unique_ptr<BaseContinuationCondition> continuationCondition)
  : BaseMotion(_robot, std::move(continuationCondition)), fileName(_fileName)
{
  Logger::info("Snapshot生成");
}

void Snapshot::executeStep()
{
  Logger::printfLog(Logger::INFO, "Requesting snapshot: %s", fileName.c_str());

  CameraServer::SnapshotActionRequest request;
  request.command = CameraServer::Command::TAKE_SNAPSHOT;

  strncpy(request.fileName, fileName.c_str(), sizeof(request.fileName) - 1);
  request.fileName[sizeof(request.fileName) - 1] = '\0';

  CameraServer::SnapshotActionResponse response;

  bool success = robot.getCameraSocketClientInstance().executeSnapshotAction(request, response);

  if(success) {
    Logger::info("Snapshot:撮影成功");
  } else {
    Logger::error("Snapshot:撮影失敗");
  }
}