/**
 * @file   SocketClient.cpp
 * @brief  カメラサーバーと通信するクラス
 * @author okuyama0528 sadomiya-sousi
 */

#include "SocketClient.h"
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

/**
 * コンストラクタ
 * @brief ソケット番号と接続状態を初期化
 */
SocketClient::SocketClient() : sock(-1), isConnected(false) {}

/**
 * デストラクタ
 * @brief 接続中の場合はサーバーへ切断処理を行う
 */
SocketClient::~SocketClient()
{
  if(isConnected) {
    disconnectFromServer();
  }
}

/**
 * サーバーへ接続する
 * @param server_ip 接続先カメラサーバーのIPアドレス
 * @return 成功:true / 失敗:false
 */
bool SocketClient::connectToServer(const char* server_ip)
{
  // すでに接続済みの場合は再接続しない
  if(isConnected) {
    std::cout << "Already connected." << std::endl;
    return true;
  }

  // TCPソケットを作成
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if(sock < 0) {
    perror("Client: socket creation failed");
    return false;
  }

  // サーバーアドレス構造体の設定
  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);

  // IPアドレスをバイナリ形式へ変換
  if(inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0) {
    perror("Client: Invalid address/ Address not supported");
    close(sock);
    sock = -1;
    return false;
  }

  // サーバーへ接続要求を送る
  if(connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    perror("Client: Connection Failed");
    close(sock);
    sock = -1;
    return false;
  }

  std::cout << "Successfully connected to camera server." << std::endl;
  isConnected = true;
  return true;
}

/**
 * サーバーへ切断コマンドを送信し、接続を終了する
 */
void SocketClient::disconnectFromServer()
{
  if(isConnected) {
    // 切断コマンドを送信
    CameraServer::Command cmd = CameraServer::Command::DISCONNECT;
    send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);

    // ソケットをクローズして接続解除
    close(sock);
    sock = -1;
    isConnected = false;
    std::cout << "Disconnected from camera server." << std::endl;
  }
}

/**
 * サーバーをシャットダウンさせるコマンドを送信する
 */
void SocketClient::shutdownServer()
{
  if(isConnected) {
    // シャットダウンコマンドを送信
    CameraServer::Command cmd = CameraServer::Command::SHUTDOWN;
    send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);

    // 接続を終了
    close(sock);
    sock = -1;
    isConnected = false;
    std::cout << "Shutdown camera server." << std::endl;
  }
}

/**
 * 汎用リクエスト送受信処理
 * @tparam Req 送信するリクエストデータ型
 * @tparam Res 受信するレスポンスデータ型
 * @param request サーバーへ送るデータ
 * @param response サーバーから受け取るデータ
 * @return 成功:true / 失敗:false
 * @brief requestを送信 → responseを受信するRPC風通信処理
 */
template <typename Req, typename Res>
bool SocketClient::executeAction(const Req& request, Res& response)
{
  // 未接続なら通信不可
  if(!isConnected) {
    std::cerr << "Not connected to server." << std::endl;
    return false;
  }

  // リクエストを送信する
  if(send(sock, reinterpret_cast<const char*>(&request), sizeof(request), 0) < 0) {
    perror("Client: send failed");
    return false;
  }

  // 結果を受信する
  ssize_t bytesRead = recv(sock, reinterpret_cast<char*>(&response), sizeof(response), 0);
  if(bytesRead < 0) {
    perror("Client: recv failed");
    return false;
  } else if(bytesRead != sizeof(response)) {
    std::cerr << "Client: received incomplete response." << std::endl;
    return false;
  }

  return true;
}