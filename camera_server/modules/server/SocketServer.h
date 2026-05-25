/**
 * @file SocketServer.h
 * @brief 接続を待ち、クライアントからのリクエストを処理するクラス
 * @author okuyama0528, sadomiya-sousi
 */

#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H

#include <vector>

class SocketServer {
 public:
  /**
   * @brief SocketServerのコンストラクタ
   * @details
   * ソケットサーバーの初期状態を設定する
   */
  SocketServer();

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
  int listenSocket;  // サーバーのリッスンソケット
  bool isRunning;    // 動作状態を管理

  /**
   * @brief クライアントとの接続を処理する
   * @param clientSocket クライアントソケット
   */
  void handle_connection(int clientSocket);
};

#endif  // SOCKET_SERVER_H