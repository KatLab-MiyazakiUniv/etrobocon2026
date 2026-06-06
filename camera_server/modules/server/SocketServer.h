/**
 * @file SocketServer.h
 * @brief ソケット通信のServerクラス
 * @author okuyama0528, sadomiya-sousi
 */

#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H

#include <string>
#include "SocketProtocol.h"
#include "RealNetworkSystem.h"
#include "Logger.h"
#include <iostream>
#include <cstring>

class SocketServer {
 public:
  /**
   * @brief SocketServerのコンストラクタ
   * @param _netSys 注入する具象クラス
   * @param _port デフォルトはローカルアドレス
   */
  explicit SocketServer(INetworkSystem& _netSys = CameraServer::real,
                        int _port = CameraServer::DEFAULT_PORT);

  /**
   * @brief SocketServerのデストラクタ
   */
  ~SocketServer();

  /**
   * @brief サーバーを初期化する
   * @return true 初期化に成功した場合
   * @return false 初期化に失敗した場合
   */
  bool init();

  /**
   * @brief サーバーを実行し、クライアントからの接続を待機する
   */
  void run();

  /**
   * @brief サーバーをシャットダウンする
   */
  void shutdown();

 public:
  INetworkSystem& netSys;  // 注入される具象クラスのポインタ
  int listenSocket;        // Severのファイルディスクリプタ
  bool isRunning;          // Serverが稼働中ならtrue
  int port;                // サーバのポート番号
  static constexpr int DEFAULT_BUFLEN = 512;

  /**
   * @brief クライアントとの接続を処理する
   * @param clientSocket クライアントソケット
   */
  void handleConnection(int clientSocket);
};
#endif  // SOCKET_SERVER_H