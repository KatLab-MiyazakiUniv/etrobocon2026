/**
 * @file SocketServer.cpp
 * @brief 接続を待ち、クライアントからのリクエストを処理するクラス
 * @author sadomiya-sousi
 */

#include "SocketServer.h"

SocketServer::SocketServer(INetworkSystem& _netSys, int _port)
  : netSys(_netSys), listenSocket(-1), isRunning(false), port(_port)
{
  LOG_CREATE("SocketServer");
  Logger::printfLog(Logger::INFO, "ポート番号は%d", _port);
}

SocketServer::~SocketServer()
{
  LOG_DESTROY("SocketClient");
  shutdown();
}

bool SocketServer::init()
{
  listenSocket = netSys.socket(AF_INET, SOCK_STREAM, 0);
  if(listenSocket < 0) {
    Logger::error("init: socket()失敗");
    return false;
  }

  int opt = 1;

  if(netSys.setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    Logger::error("init: setsockopt()失敗");
    netSys.close(listenSocket);
    return false;
  }

  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  serv_addr.sin_port = htons(this->port);

  if(netSys.bind(listenSocket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    netSys.close(listenSocket);
    return false;
  }

  if(netSys.listen(listenSocket, 3) < 0) {
    Logger::error("init: listen()失敗");
    netSys.close(listenSocket);
    return false;
  }

  Logger::info("init: 起動成功");
  isRunning = true;
  return true;
}

void SocketServer::run()
{
  while(isRunning) {
    int clientSocket = netSys.accept(listenSocket, (struct sockaddr*)NULL, NULL);
    if(clientSocket < 0) {
      Logger::debug("run: accept()失敗");
      if(!isRunning) {
        break;
      }
      continue;
    }
    handleConnection(clientSocket);
    netSys.close(clientSocket);
  }
}

void SocketServer::shutdown()
{
  isRunning = false;
  if(listenSocket != -1) {
    netSys.close(listenSocket);
    listenSocket = -1;
  }
  Logger::debug("shutdown:Socket server Shutdown");
}

void SocketServer::handleConnection(int clientSocket)
{
  char recvbuf[SocketServer::DEFAULT_BUFLEN];
  ssize_t iResult;

  // クライアントからのデータ受信ループ
  do {
    iResult = netSys.recv(clientSocket, recvbuf, SocketServer::DEFAULT_BUFLEN, 0);
    if(iResult > 0) {
      if(static_cast<size_t>(iResult) == CameraServer::COMMAND_SIZE) {
        CameraServer::Command cmd = *reinterpret_cast<CameraServer::Command*>(recvbuf);
        switch(cmd) {
          case CameraServer::Command::SHUTDOWN:
            shutdown();
            return;
          case CameraServer::Command::DISCONNECT:
            // クライアントからの切断要求なのでreturn
            return;
          default:
            break;
        }
      } else {
        Logger::printfLog(Logger::ERROR, "handleConnection: Received unexpected data size: %zd",
                          (ssize_t)iResult);
      }
    } else if(iResult == 0) {
    } else {
      if(isRunning) {
        Logger::error("handleConnection: recv failed");
      }
    }
  } while(iResult > 0);
}
