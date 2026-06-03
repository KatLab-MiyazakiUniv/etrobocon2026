/**
 * @file   SocketClient.h
 * @brief  カメラサーバーと通信するクラス
 * @author sadomiya-sousi, okuyama0528
 */

#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include "RealNetworkSystem.h"
#include "MockNetworkSystem.h"
#include "SocketProtocol.h"
#include "Logger.h"
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

class SocketClient {
 public:
  /**
   * @brief コンストラクタ
   */
  explicit SocketClient(INetworkSystem* networkSystem = &real, int port = 27015,
                        const char* server_ip = "127.0.0.1");

  /**
   * @brief デストラクタ
   */
  virtual ~SocketClient();

  /**
   * @brief サーバーに接続する
   * @param server_ip サーバーのIPアドレス
   * @return true 接続に成功した場合
   * @return false 接続に失敗した場合
   */
  virtual bool connectToServer();

  /**
   * @brief サーバーから切断する
   */
  virtual void disconnectFromServer();

  /**
   * @brief サーバーをシャットダウンする
   */
  virtual void shutdownServer();

 protected:
  INetworkSystem* netSys;
  int sock;              // ソケットファイルディスクリプタ
  bool isConnected;      // サーバーへの接続状態
  int port;              // サーバーのポート番号
  std::string serverIp;  // サーバーのIPアドレス

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