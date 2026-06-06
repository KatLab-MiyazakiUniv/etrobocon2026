/**
 * @file    SocketClient.cpp
 * @brief   カメラサーバーと通信するクラス
 * @author  sadomiya-sousi, okuyama0528
 */

#include "SocketClient.h"

SocketClient::SocketClient(INetworkSystem& _netSys, int _port, const char* _serverIp)
  : netSys(_netSys), sock(-1), isConnected(false), port(_port), serverIp(_serverIp)
{
  LOG_CREATE("SocketClient");
  Logger::printfLog(Logger::INFO, "ポート番号は%d,ipは%s ", _port, _serverIp);
}

SocketClient::~SocketClient()
{
  LOG_DESTROY("SocketClient");
  if(isConnected) {
    disconnectFromServer();
  }
}

bool SocketClient::connectToServer()
{
  if(isConnected) {
    return true;
  }

  sock = netSys.socket(AF_INET, SOCK_STREAM, 0);
  if(sock < 0) {
    Logger::error("connectToServer: socket()失敗");
    return false;
  }

  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(this->port);
  if(inet_pton(AF_INET, serverIp.c_str(), &serv_addr.sin_addr) <= 0) {
    Logger::printfLog(Logger::ERROR, "connectToServer: inet_pton()失敗:ソケット %d ", sock);
    netSys.close(sock);
    sock = -1;
    return false;
  }

  if(netSys.connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    Logger::printfLog(Logger::ERROR, "connectToServer: connect():ソケット %d を閉じます", sock);
    netSys.close(sock);
    sock = -1;
    return false;
  }

  isConnected = true;
  return true;
}

void SocketClient::disconnectFromServer()
{
  if(isConnected) {
    CameraServer::Command cmd = CameraServer::Command::DISCONNECT;
    netSys.send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);
    netSys.close(sock);
    sock = -1;
    isConnected = false;
  } else {
    Logger::info("disconnectFromServer: 終了");
  }
}

void SocketClient::shutdownServer()
{
  if(isConnected) {
    CameraServer::Command cmd = CameraServer::Command::SHUTDOWN;

    netSys.send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);
    netSys.close(sock);

    sock = -1;
    isConnected = false;
  } else {
    Logger::info("shutdownServer: 未接続状態のためスキップ");
  }
  Logger::info("shutdownServer: 終了");
}
