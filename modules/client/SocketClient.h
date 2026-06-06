/**
 * @file   SocketClient.h
 * @brief  カメラサーバーと通信するクラス
 * @author sadomiya-sousi, okuyama0528
 */

#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include "RealNetworkSystem.h"
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
  explicit SocketClient(INetworkSystem& _netSys = CameraServer::real,
                        int _port = CameraServer::DEFAULT_PORT,
                        const char* _serverIp = "127.0.0.1");

  /**
   * @brief デストラクタ
   */
  virtual ~SocketClient();

  /**
   * @brief サーバーに接続する
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
  INetworkSystem& netSys;
  int sock;              // ソケットファイルディスクリプタ
  bool isConnected;      // サーバーへの接続状態
  int port;              // サーバーのポート番号
  std::string serverIp;  // サーバーのIPアドレス
};

#endif  // SOCKET_CLIENT_H