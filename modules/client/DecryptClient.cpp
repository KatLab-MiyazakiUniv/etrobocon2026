/**
 * @file   DecryptClient.cpp
 * @brief  復号サーバーと通信するクラス
 * @author HaruArima08
 */

#include "DecryptClient.h"

DecryptClient::DecryptClient(INetworkSystem& _netSys, int _port, const char* _serverIp)
  : netSys(_netSys), sock(-1), isConnected(false), port(_port), serverIp(_serverIp)
{
  LOG_CREATE("DecryptClient");
  Logger::printfLog(Logger::INFO, "DecryptClient:ポート番号は%d,ipは%s", _port, _serverIp);
}

DecryptClient::~DecryptClient()
{
  LOG_DESTROY("DecryptClient");
  if(isConnected) {
    disconnectFromServer();
  }
}

bool DecryptClient::connectToServer()
{
  if(isConnected) {
    return true;
  }

  sock = netSys.socket(AF_INET, SOCK_STREAM, 0);
  if(sock < 0) {
    Logger::error("DecryptClient:connectToServer: socket()失敗");
    return false;
  }

  struct sockaddr_in servAddr;
  memset(&servAddr, 0, sizeof(servAddr));
  servAddr.sin_family = AF_INET;
  servAddr.sin_port = htons(this->port);
  if(inet_pton(AF_INET, serverIp.c_str(), &servAddr.sin_addr) <= 0) {
    Logger::printfLog(Logger::ERROR, "DecryptClient:connectToServer: inet_pton()失敗:ソケット %d",
                      sock);
    netSys.close(sock);
    sock = -1;
    return false;
  }

  if(netSys.connect(sock, (struct sockaddr*)&servAddr, sizeof(servAddr)) < 0) {
    Logger::printfLog(Logger::ERROR,
                      "DecryptClient:connectToServer: connect()失敗:ソケット %d を閉じます", sock);
    netSys.close(sock);
    sock = -1;
    return false;
  }

  isConnected = true;
  return true;
}

void DecryptClient::disconnectFromServer()
{
  if(isConnected) {
    DecryptServer::Command command = DecryptServer::Command::DISCONNECT;
    netSys.send(sock, &command, DecryptServer::COMMAND_SIZE, 0);
    netSys.close(sock);
    sock = -1;
    isConnected = false;
  } else {
    Logger::info("DecryptClient:disconnectFromServer: 未接続状態のためスキップ");
  }
}

void DecryptClient::shutdownServer()
{
  if(isConnected) {
    DecryptServer::Command command = DecryptServer::Command::SHUTDOWN;
    netSys.send(sock, &command, DecryptServer::COMMAND_SIZE, 0);
    netSys.close(sock);
    sock = -1;
    isConnected = false;
  } else {
    Logger::info("DecryptClient:shutdownServer: 未接続状態のためスキップ");
  }
}

bool DecryptClient::executeDecrypt(const DecryptServer::DecryptRequest& request,
                                   DecryptServer::DecryptResponse& response)
{
  return executeAction(request, response);
}

bool DecryptClient::executePing(const DecryptServer::PingRequest& request,
                                DecryptServer::PingResponse& response)
{
  return executeAction(request, response);
}

int DecryptClient::getSock() const
{
  return sock;
}

bool DecryptClient::getIsConnected() const
{
  return isConnected;
}

int DecryptClient::getPort() const
{
  return port;
}

void DecryptClient::setPort(int _port)
{
  port = _port;
}

const std::string& DecryptClient::getServerIp() const
{
  return serverIp;
}

void DecryptClient::setServerIp(const std::string& _serverIp)
{
  serverIp = _serverIp;
}
