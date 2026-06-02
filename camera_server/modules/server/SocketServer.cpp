/**
 * @file SocketServer.cpp
 * @brief 接続を待ち、クライアントからのリクエストを処理するクラス
 * @author sadomiya-sousi
 */

#include "SocketServer.h"

SocketServer::SocketServer(INetworkSystem* networkSystem, int port)
  : netSys(networkSystem), listenSocket(-1), isRunning(false), port(port)
{
  LOG_CREATE("SocketServer");
}

bool SocketServer::init()
{
  Logger::info("init: 開始");
  listenSocket = netSys->socket(AF_INET, SOCK_STREAM, 0);
  if(listenSocket < 0) {
    Logger::error("init: socket()失敗");
    return false;
  }

  int opt = 1;

  Logger::info("init: setsockopt()実行前");
  if(netSys->setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    Logger::error("init: setsockopt()失敗");

    // close()が成功するか
    if(netSys->close(listenSocket) < 0) {
      Logger::error("init: close()失敗");
    }
    return false;
  }

  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 全インターフェースで待受
  serv_addr.sin_port = htons(this->port);         // ポート設定

  Logger::info("init: bind()実行前");
  if(netSys->bind(listenSocket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    Logger::error("init: bind()失敗");

    // close()が成功するか
    if(netSys->close(listenSocket) < 0) {
      Logger::error("init: close()失敗");
    }
    return false;
  }

  Logger::info("init: listen()実行前");
  if(netSys->listen(listenSocket, 3) < 0) {
    // std::cerr << "[ERROR] listen failed" << std::endl;
    Logger::error("init: listen()失敗");
    // close()が成功するか
    if(netSys->close(listenSocket) < 0) {
      Logger::error("init: close()失敗");
    }
    return false;
  }

  Logger::info("init: 起動成功");
  // 稼働フラグを true に
  isRunning = true;
  return true;
}

void SocketServer::run()
{
  Logger::info("run: 開始");
  while(isRunning) {
    int clientSocket = netSys->accept(listenSocket, (struct sockaddr*)NULL, NULL);
    if(clientSocket < 0) {
      Logger::debug("run: accept()失敗");
      if(!isRunning) {
        Logger::debug("run: サーバー停止");
        break;
      }
      Logger::debug("run: サーバー継続");
      continue;
    }
    Logger::info("run: Client connected.");
    handleConnection(clientSocket);
    netSys->close(clientSocket);
    Logger::info("run: Client disconnected.");
  }
}

void SocketServer::shutdown()
{
  Logger::info("shutdown: 開始");
  isRunning = false;
  if(listenSocket != -1) {
    netSys->close(listenSocket);
    listenSocket = -1;
  }
  Logger::info("shutdown: Socket server shutting down.");
}

void SocketServer::handleConnection(int clientSocket)
{
  char recvbuf[SocketServer::DEFAULT_BUFLEN];
  ssize_t iResult;

  // クライアントからのデータ受信ループ
  do {
    iResult = netSys->recv(clientSocket, recvbuf, SocketServer::DEFAULT_BUFLEN, 0);
    if(iResult > 0) {
      // 受信データが最低限コマンドサイズの4バイトあるか確認
      if(static_cast<size_t>(iResult) >= sizeof(CameraServer::Command)) {
        // 先頭をコマンドとして解釈
        // ポインタ変数に変換した後にポインタ変数をデリファレンスすることで実際の値を参照
        CameraServer::Command cmd = *reinterpret_cast<CameraServer::Command*>(recvbuf);

        switch(cmd) {
          case CameraServer::Command::SHUTDOWN:
            Logger::info("handleConnection: Received SHUTDOWN command.");
            shutdown();
            return;
          case CameraServer::Command::DISCONNECT:
            Logger::info("handleConnection: Received DISCONNECT command.");
            // クライアントからの切断要求なので return
            return;
          default:
            Logger::info("handleConnection: Received command (ignored).");
            break;
        }
      } else {
        Logger::printfLog(Logger::ERROR, "handleConnection: Received unexpected data size: %zd",
                          (ssize_t)iResult);
      }
    } else if(iResult == 0) {
      Logger::info("handleConnection: Connection closing...");
    } else {
      if(isRunning) {
        Logger::error("handleConnection: recv failed");
      }
    }
  } while(iResult > 0);
}
