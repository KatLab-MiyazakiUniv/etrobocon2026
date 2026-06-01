/**
 * @file SocketServer.h
 * @brief ソケット通信のServerクラス
 * @author okuyama0528, sadomiya-sousi
 */

#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H

#include <string>
#include "SocketProtocol.h"
#include "INetworkSystem.h"
#include "Logger.h"

class SocketServer {
 public:
  /**
   * @brief SocketServerのコンストラクタ
   * @details
   * ソケットサーバーの初期状態を設定する
   */
  explicit SocketServer(INetworkSystem* networkSystem);

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

 private:
  INetworkSystem* netSys;  // 注入される具象クラスのポインタ
  int listenSocket;        // Severのファイルディスクリプタ
  bool isRunning;          // Serverが稼働中ならtrue

  /**
   * @brief クライアントとの接続を処理する
   * @param clientSocket クライアントソケット
   */
  void handle_connection(int clientSocket);
};

#endif  // SOCKET_SERVER_H