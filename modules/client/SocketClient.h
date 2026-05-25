/**
 * @file   SocketClient.h
 * @brief  カメラサーバーと通信するクラス
 * @author okuyama0528 sadomiya-sousi
 */

#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include "SocketProtocol.h"
#define PORT 27015             // サーバーのポート番号
#define SERVER_IP "127.0.0.1"  // サーバーのIPアドレス

class SocketClient {
 public:
  /**
   * @brief コンストラクタ
   *
   */
  SocketClient();

  /**
   * @brief デストラクタ
   *
   */
  virtual ~SocketClient();

  /**
   * @brief サーバーに接続する
   * @param server_ip サーバーのIPアドレス
   * @return true 接続に成功した場合
   * @return false 接続に失敗した場合
   */
  virtual bool connectToServer(const char* server_ip = SERVER_IP);

  /**
   * @brief サーバーから切断する
   *
   */
  virtual void disconnectFromServer();

  /**
   * @brief サーバーをシャットダウンする
   *
   */
  virtual void shutdownServer();

 protected:
  int sock;          // ソケットファイルディスクリプタ
  bool isConnected;  // サーバーへの接続状態

 private:
  /**
   * @brief サーバーにリクエストを送信し、レスポンスを受信するtemplate関数
   * @tparam Req リクエストの型
   * @tparam Res レスポンスの型
   * @param request 送信するリクエスト
   * @param response 受信するレスポンス
   * @return true 送受信に成功した場合
   * @return false 送受信に失敗した場合
   */
  template <typename Req, typename Res>
  bool executeAction(const Req& request, Res& response);
};

#endif  // SOCKET_CLIENT_H