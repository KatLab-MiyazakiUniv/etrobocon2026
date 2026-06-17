/**
 * @file   Snapshot.cpp
 * @brief  サーバーに写真撮影を依頼するクラス
 * @author sadomiya-sousi
 */

#include "Snapshot.h"
#include "SocketClient.h"
#include <iostream>
#include <string.h>
#include "Logger.h"

Snapshot::Snapshot(Robot& _robot, const std::string& _fileName) : robot(_robot), fileName(_fileName)
{
}

void Snapshot::run()
{
  Logger::printfLog(Logger::INFO, "Requesting snapshot: %s", fileName.c_str());

  SocketClient& client = robot.getCameraSocketClientInstance();

  CameraServer::SnapshotActionRequest request;
  request.command = CameraServer::Command::TAKE_SNAPSHOT;

  strncpy(request.fileName, fileName.c_str(), sizeof(request.fileName) - 1);
  request.fileName[sizeof(request.fileName) - 1] = '\0';

  CameraServer::SnapshotActionResponse response;

  bool success = client.executeSnapshotAction(request, response);
  if(success) {
    Logger::info("Snapshot taken successfully.");
  } else {
    Logger::error("Server failed to take snapshot.");
  }
}