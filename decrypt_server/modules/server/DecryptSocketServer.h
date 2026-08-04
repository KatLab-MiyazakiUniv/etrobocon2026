/**
 * @file   DecryptSocketServer.h
 * @brief  接続を待ち、クライアントからの復号リクエストを処理するクラス
 * @author HaruArima08
 */

#ifndef DECRYPT_SOCKET_SERVER_H
#define DECRYPT_SOCKET_SERVER_H

#include "DecryptProtocol.h"
#include "DecryptActionHandler.h"
#include "RealNetworkSystem.h"
#include "SocketIo.h"
#include "Logger.h"
#include <cstring>

class DecryptSocketServer {
 public:
  /**
   * @brief コンストラクタ
   * @param _decryptHandler 復号アクションハンドラ
   * @param _netSys 注入する具象クラス
   * @param _port デフォルトは27016
   */
  explicit DecryptSocketServer(DecryptActionHandler& _decryptHandler, INetworkSystem& _netSys,
                               int _port = DecryptServer::DEFAULT_PORT);

  /**
   * @brief デストラクタ
   */
  ~DecryptSocketServer();

  /**
   * @brief サーバーを初期化する
   * @return true 初期化に成功した場合
   * @return false 初期化に失敗した場合
   */
  bool init();

  /**
   * @brief サーバーを接続受け入れ状態にし、接続を待機する
   */
  void run();

  /**
   * @brief サーバーをシャットダウンする
   */
  void shutdown();

  /**
   * @brief クライアントとの接続を処理する
   * @param clientSocket クライアントソケット
   */
  void handleConnection(int clientSocket);

  /**
   * @brief サーバーのリッスンソケットを取得する
   * @return int サーバーのリッスンソケットのファイルディスクリプタ
   */
  int getListenSocket() const;

  /**
   * @brief サーバーが稼働中かどうかを取得する
   * @return true サーバーが稼働中の場合
   * @return false サーバーが停止している場合
   */
  bool getIsRunning() const;

  /**
   * @brief サーバーのポート番号を取得する
   * @return int サーバーのポート番号
   */
  int getPort() const;

  /**
   * @brief サーバーのポート番号を設定する
   * @param _port 設定するポート番号
   */
  void setPort(int _port);

  /**
   * @brief ネットワークシステムを取得する
   * @return INetworkSystem& ネットワークシステムへの参照
   */
  INetworkSystem& getNetSys() const;

 private:
  INetworkSystem& netSys;                   // 注入される具象クラスへの参照
  int listenSocket;                         // サーバーのファイルディスクリプタ(セッター無し)
  bool isRunning;                           // サーバーが稼働中ならtrue(セッター無し)
  int port;                                 // サーバーのポート番号
  DecryptActionHandler& decryptHandler;     // 復号アクションのハンドラ
  static constexpr int LISTEN_BACKLOG = 3;  // 接続待ちキューの最大長

  /**
   * @brief 復号コマンドを処理する
   * @param clientSocket クライアントソケット
   * @return true 処理に成功した場合
   * @return false 受信または送信に失敗した場合
   */
  bool handleDecryptCommand(int clientSocket);

  /**
   * @brief Pingコマンドを処理する
   * @param clientSocket クライアントソケット
   * @return true 処理に成功した場合
   * @return false 受信または送信に失敗した場合
   */
  bool handlePingCommand(int clientSocket);
};

#endif  // DECRYPT_SOCKET_SERVER_H
