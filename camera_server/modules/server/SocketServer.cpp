/**
 * @file SocketServer.cpp
 * @brief 接続を待ち、クライアントからのリクエストを処理するクラス
 * @author sadomiya-sousi
 */

#include "SocketServer.h"
#include "Logger.h"
#include <string>

#define PORT 27015
#define DEFAULT_BUFLEN 512

SocketServer::SocketServer(INetworkSystem* networkSystem)
  : netSys(networkSystem), listenSocket(-1), isRunning(false)
{
}

bool SocketServer::init()
{
  listenSocket = netSys->socket(AF_INET, SOCK_STREAM, 0);
  if(listenSocket < 0) {
    Logger::error("socket failed");
    return false;
  }

  int opt = 1;
  if(netSys->setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    Logger::error("setsockopt failed");
    netSys->close(listenSocket);
    return false;
  }

  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 全インターフェースで待受
  serv_addr.sin_port = htons(PORT);               // ポート設定

  if(netSys->bind(listenSocket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    Logger::error("bind failed");
    netSys->close(listenSocket);
    return false;
  }

  if(netSys->listen(listenSocket, 3) < 0) {
    Logger::error("listen failed");
    netSys->close(listenSocket);
    return false;
  }

  Logger::info("Socket server initialized and listening on port " + std::to_string(PORT));
  return true;
}

void SocketServer::run()
{
  isRunning = true;
  while(isRunning) {
    int clientSocket = netSys->accept(listenSocket, (struct sockaddr*)NULL, NULL);
    if(clientSocket < 0) {
      if(!isRunning) break;
      Logger::error("accept failed");
      continue;
    }
    Logger::info("Client connected.");

    handle_connection(clientSocket);

    netSys->close(clientSocket);
    Logger::info("Client disconnected.");
  }
}

void SocketServer::shutdown()
{
  isRunning = false;
  if(listenSocket != -1) {
    netSys->close(listenSocket);
    listenSocket = -1;
  }
  Logger::info("Socket server shutting down.");
}

void SocketServer::handle_connection(int clientSocket)
{
  char recvbuf[DEFAULT_BUFLEN];
  ssize_t iResult;

  // クライアントからのデータ受信ループ
  do {
    iResult = netSys->recv(clientSocket, recvbuf, DEFAULT_BUFLEN, 0);
    if(iResult > 0) {
      // 受信データが最低限コマンドサイズあるか確認
      if(static_cast<size_t>(iResult) >= sizeof(CameraServer::Command)) {
        // 先頭をコマンドとして解釈
        CameraServer::Command cmd = *reinterpret_cast<CameraServer::Command*>(recvbuf);

        switch(cmd) {
          case CameraServer::Command::SHUTDOWN:
            Logger::info("Received SHUTDOWN command.");

            // サーバー全体停止
            shutdown();
            break;

          case CameraServer::Command::DISCONNECT:
            Logger::info("Received DISCONNECT command.");
            // クライアントからの切断要求なのでreturn
            return;

          default:
            Logger::info("Received command (ignored).");
            break;
        }
      } else {
        Logger::error("Received unexpected data size: " + std::to_string(iResult));
      }
    } else if(iResult == 0) {
      // クライアントが正常に接続終了
      Logger::info("Connection closing...");
    } else {
      // recvエラー
      if(isRunning) Logger::error("recv failed");
    }
  } while(iResult > 0);
}
