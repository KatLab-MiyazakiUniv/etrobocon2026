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

bool SocketClient::executeColorRegionDetection(
    const CameraServer::ColorRegionDetectorRequest& request,
    CameraServer::ColorRegionDetectorResponse& response)
{
  return executeAction(request, response);
}

bool SocketClient::executeSnapshotAction(const CameraServer::SnapshotActionRequest& request,
                                         CameraServer::SnapshotActionResponse& response)
{
  return executeAction(request, response);
}

bool SocketClient::executeGetDecryptionKey(const CameraServer::DecryptionKeyRequest& request,
                                           CameraServer::DecryptionKeyResponse& response)
{
  return executeAction(request, response);
}

template <typename Req, typename Res>
bool SocketClient::executeAction(const Req& request, Res& response)
{
  if(!isConnected) {
    Logger::error("Not connected to server");
    return false;
  }

  // リクエストを送信する
  if(netSys.send(sock, reinterpret_cast<const char*>(&request), sizeof(request), 0) < 0) {
    Logger::error("Client: send failed");
    return false;
  }

  // 結果を受信する
  ssize_t bytesRead = netSys.recv(sock, reinterpret_cast<char*>(&response), sizeof(response), 0);
  if(bytesRead < 0) {
    Logger::error("Client: recv failed");
    return false;
  } else if(bytesRead != sizeof(response)) {
    Logger::error("Client: received incomplete response");
    return false;
  }
  return true;
}