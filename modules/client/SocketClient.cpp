/**
 * @file    SocketClient.cpp
 * @brief   カメラサーバーと通信するクラス
 * @author  sadomiya-sousi okuyama0528
 */

#include "SocketClient.h"

SocketClient::SocketClient(INetworkSystem& _netSys, int _port, const char* _serverIp)
  : netSys(_netSys), sock(-1), isConnected(false), port(_port), serverIp(_serverIp)
{
  LOG_CREATE("SocketClient");
  Logger::printfLog(Logger::INFO, "SocketClient:ポート番号は%d,ipは%s", _port, _serverIp);
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
    Logger::error("SocketClient:connectToServer: socket()失敗");
    return false;
  }

  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(this->port);
  if(inet_pton(AF_INET, serverIp.c_str(), &serv_addr.sin_addr) <= 0) {
    Logger::printfLog(Logger::ERROR, "SocketClient:connectToServer: inet_pton()失敗:ソケット %d",
                      sock);
    netSys.close(sock);
    sock = -1;
    return false;
  }

  if(netSys.connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    Logger::printfLog(Logger::ERROR,
                      "SocketClient:connectToServer: connect():ソケット %d を閉じます", sock);
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
    Logger::info("SocketClient:disconnectFromServer: 終了");
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
    Logger::info("SocketClient:shutdownServer: 未接続状態のためスキップ");
  }
  Logger::info("SocketClient:shutdownServer: 終了");
}

bool SocketClient::executeColorRegionDetection(
    const CameraServer::ColorRegionDetectorRequest& request,
    CameraServer::ColorRegionDetectorResponse& response)
{
  return executeAction(request, response);
}

template <typename Req, typename Res>
bool SocketClient::executeAction(const Req& request, Res& response)
{
  if(!isConnected) {
    Logger::error("SocketClient:サーバーに接続されていません");
    return false;
  }

  // リクエストを送信する
  if(netSys.send(sock, reinterpret_cast<const char*>(&request), sizeof(request), 0) < 0) {
    Logger::error("SocketClient:送信失敗");
    return false;
  }

  // 結果を受信する
  ssize_t bytesRead = netSys.recv(sock, reinterpret_cast<char*>(&response), sizeof(response), 0);
  if(bytesRead < 0) {
    Logger::error("SocketClient:受信失敗");
    return false;
  } else if(bytesRead != sizeof(response)) {
    Logger::error("SocketClient:不完全なレスポンスを受信しました");
    return false;
  }
  return true;
}

int SocketClient::getSock() const
{
  return sock;
}

bool SocketClient::getIsConnected() const
{
  return isConnected;
}

int SocketClient::getPort() const
{
  return port;
}

void SocketClient::setPort(int _port)
{
  port = _port;
}

const std::string& SocketClient::getServerIp() const
{
  return serverIp;
}

void SocketClient::setServerIp(const std::string& _serverIp)
{
  serverIp = _serverIp;
}