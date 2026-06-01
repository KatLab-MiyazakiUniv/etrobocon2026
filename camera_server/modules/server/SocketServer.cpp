/**
 * @file SocketServer.cpp
 * @brief 接続を待ち、クライアントからのリクエストを処理するクラス
 * @author sadomiya-sousi
 */

#include "SocketServer.h"
#include <string>
#include <iostream>
#include <cstring>

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
    std::cerr << "[ERROR] socket failed" << std::endl;
    return false;
  }

  int opt = 1;
  if(netSys->setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    std::cerr << "[ERROR] setsockopt failed" << std::endl;
    // close()が成功したか
    if(netSys->close(listenSocket) < 0) {
      std::cerr << "[ERROR] close failed" << std::endl;
    }
    return false;
  }

  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 全インターフェースで待受
  serv_addr.sin_port = htons(PORT);               // ポート設定

  if(netSys->bind(listenSocket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    std::cerr << "[ERROR] bind failed" << std::endl;
    // close()が成功するか
    if(netSys->close(listenSocket) < 0) {
      std::cerr << "[ERROR] close failed" << std::endl;
    }
    return false;
  }

  if(netSys->listen(listenSocket, 3) < 0) {
    std::cerr << "[ERROR] listen failed" << std::endl;
    // close()が成功するか
    if(netSys->close(listenSocket) < 0) {
      std::cerr << "[ERROR] close failed" << std::endl;
    }
    return false;
  }
  // serverが起動して始めてisRunningをtrueにする
  isRunning = true;

  std::cout << "[INFO] Socket server initialized and listening on port " << PORT << std::endl;
  return true;
}

void SocketServer::run()
{
  while(isRunning) {
    int clientSocket = netSys->accept(listenSocket, (struct sockaddr*)NULL, NULL);
    if(clientSocket < 0) {
      if(!isRunning) break;
      std::cerr << "[ERROR] accept failed" << std::endl;
      continue;
    }
    std::cout << "[INFO] Client connected." << std::endl;

    handle_connection(clientSocket);

    netSys->close(clientSocket);
    std::cout << "[INFO] Client disconnected." << std::endl;
  }
}

void SocketServer::shutdown()
{
  isRunning = false;
  if(listenSocket != -1) {
    netSys->close(listenSocket);
    listenSocket = -1;
  }
  std::cout << "[INFO] Socket server shutting down." << std::endl;
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
            std::cout << "[INFO] Received SHUTDOWN command." << std::endl;

            // サーバー全体停止
            shutdown();
            break;

          case CameraServer::Command::DISCONNECT:
            std::cout << "[INFO] Received DISCONNECT command." << std::endl;
            // クライアントからの切断要求なのでreturn
            return;

          default:
            std::cout << "[INFO] Received command (ignored)." << std::endl;
            break;
        }
      } else {
        std::cerr << "[ERROR] Received unexpected data size: " << iResult << std::endl;
      }
    } else if(iResult == 0) {
      // クライアントが正常に接続終了
      std::cout << "[INFO] Connection closing..." << std::endl;
    } else {
      // recvエラー
      if(isRunning) {
        std::cerr << "[ERROR] recv failed" << std::endl;
      }
    }
  } while(iResult > 0);
}