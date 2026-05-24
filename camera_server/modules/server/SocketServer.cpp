/**
 * @file SocketServer.cpp
 * @brief 接続を待ち、クライアントからのリクエストを処理するクラス
 * @author okuyama0528, sadomiya-sousi
 */

#include "SocketServer.h"
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "SocketProtocol.h"

#define PORT 27015
#define DEFAULT_BUFLEN 512

/**
 * コンストラクタ
 * @brief 初期状態としてソケット未生成・未起動に設定
 */
SocketServer::SocketServer() : listenSocket(-1), isRunning(false) {}

/**
 * サーバー初期化処理
 * @brief ソケット生成・bind・listenまでを行う
 * @return 成功:true / 失敗:false
 */
bool SocketServer::init()
{
  // サーバー用ソケット生成
  listenSocket = socket(AF_INET, SOCK_STREAM, 0);
  if(listenSocket < 0) {
    perror("socket failed");
    return false;
  }

  // ソケットの再利用を許可（再起動時のbind失敗を防ぐ）
  int opt = 1;
  if(setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    perror("setsockopt failed");
    close(listenSocket);
    return false;
  }

  // サーバーアドレス設定（全IPから接続受付）
  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 全インターフェースで待受
  serv_addr.sin_port = htons(PORT);               // ポート設定

  // ソケットをポートに紐付け
  if(bind(listenSocket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    perror("bind failed");
    close(listenSocket);
    return false;
  }

  // 接続待ち状態にする

  if(listen(listenSocket, 3) < 0) {
    perror("listen failed");
    close(listenSocket);
    return false;
  }

  std::cout << "Socket server initialized and listening on port " << PORT << std::endl;
  return true;
}

/**
 * サーバー本体のループ処理
 * @brief クライアント接続を受け付け、1接続ごとに処理する
 */
void SocketServer::run()
{
  isRunning = true;
  while(isRunning) {
    // クライアント接続待ち
    int clientSocket = accept(listenSocket, (struct sockaddr*)NULL, NULL);
    if(clientSocket < 0) {
      // shutdown中にacceptが失敗するのは正常終了扱い
      if(!isRunning) break;  // Shutdown was called
      perror("accept failed");
      continue;
    }
    std::cout << "Client connected." << std::endl;

    // クライアントとの通信処理
    handle_connection(clientSocket);

    // 接続終了
    close(clientSocket);
    std::cout << "Client disconnected." << std::endl;
  }
}

/**
 * サーバー停止処理
 * @brief listenソケットを閉じてrunループを終了させる
 */
void SocketServer::shutdown()
{
  isRunning = false;

  // listenソケットを閉じることでacceptを解除
  if(listenSocket != -1) {
    close(listenSocket);
    listenSocket = -1;
  }
  std::cout << "Socket server shutting down." << std::endl;
}

/**
 * クライアントとの通信処理
 * @brief 受信データを解析し、コマンドに応じて処理を行う
 */
void SocketServer::handle_connection(int clientSocket)
{
  char recvbuf[DEFAULT_BUFLEN];
  ssize_t iResult;

  // クライアントからのデータ受信ループ
  do {
    iResult = recv(clientSocket, recvbuf, DEFAULT_BUFLEN, 0);
    if(iResult > 0) {
      // 受信データが最低限コマンドサイズあるか確認
      if(static_cast<size_t>(iResult) >= sizeof(CameraServer::Command)) {
        // 先頭をコマンドとして解釈
        CameraServer::Command cmd = *reinterpret_cast<CameraServer::Command*>(recvbuf);

        switch(cmd) {
          case CameraServer::Command::SHUTDOWN:
            std::cout << "Received SHUTDOWN command." << std::endl;

            // サーバー全体停止
            shutdown();
            break;

          case CameraServer::Command::DISCONNECT:
            std::cout << "Received DISCONNECT command." << std::endl;
            // クライアントからの切断要求なのでreturn
            return;

          default:
            std::cout << "Received command (ignored)." << std::endl;
            break;
        }
      } else {
        std::cerr << "Received unexpected data size: " << iResult << std::endl;
      }
    } else if(iResult == 0) {
      // クライアントが正常に接続終了
      std::cout << "Connection closing..." << std::endl;
    } else {
      // recvエラー
      if(isRunning) perror("recv failed");
    }
  } while(iResult > 0);
}