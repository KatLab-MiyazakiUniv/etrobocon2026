/**
 * @file   SocketClient.h
 * @brief  サーバーと通信するクラス
 * @author sadomiya-sousi, okuyama0528
 */

#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include "RealNetworkSystem.h"
#include "SocketProtocol.h"
#include "Logger.h"
#include <arpa/inet.h>
#include <string.h>

class SocketClient {
 public:
  /**
   * @brief コンストラクタ
   * @param _netSys 注入する具象クラス
   * @param _port デフォルトは27015
   * @param _serverIp デフォルトはローカルアドレス
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