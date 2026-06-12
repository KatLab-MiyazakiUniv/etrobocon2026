/**
 * @file SocketServer.h
 * @brief 接続を待ち、クライアントからのリクエストを処理するクラス
 * @author sadomiya-sousi, takuchi17
 */

#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H

#include <string>
#include "SocketProtocol.h"
#include "RealNetworkSystem.h"
#include "actions/ColorRegionDetectionActionHandler.h"
#include "Logger.h"
#include "CameraCapture.h"
#include <cstring>

class SocketServer {
 public:
  /**
   * @brief SocketServerのコンストラクタ
   * @param _netSys 注入する具象クラス
   * @param _port デフォルトは27015
   */
  explicit SocketServer(ColorRegionDetectionActionHandler& _colorRegionDetectionHandler,
                        INetworkSystem& _netSys, int _port = CameraServer::DEFAULT_PORT);

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
   * @brief サーバーを実行し接続用受け入れ状態にし,接続を待機
   */
  void run();

  /**
   * @brief サーバーのリッスンソケットを取得する
   * @return int サーバーのリッスンソケットのファイルディスクリプタ
   */
  int getListenSocket() const { return listenSocket; }

  /**
   * @brief サーバーが稼働中かどうかを取得する
   * @return bool サーバーが稼働中ならtrue, そうでないならfalse
   */
  bool getIsRunning() const { return isRunning; }

  /**
   * @brief サーバーのポート番号を取得する
   * @return int サーバーのポート番号
   */
  int getPort() const { return port; }

  /**
   * @brief デフォルトのバッファサイズを取得する
   * @return int デフォルトのバッファサイズ
   */
  static int getDefaultBufLen() { return DEFAULT_BUFLEN; }

  /**
   * @brief サーバーのリッスンソケットを設定する
   * @param socket 設定するファイルディスクリプタ
   */
  void setListenSocket(int socket) { listenSocket = socket; }

  /**
   * @brief サーバーの稼働状態を設定する
   * @param isRun 1なら稼働中, 0なら停止
   */
  void setIsRunning(int isRun) { isRunning = isRun; }

  /**
   * @brief サーバーのポート番号を設定する
   * @param portNumber 設定するポート番号
   */
  void setPort(int portNumber) { port = portNumber; }

  /**
   * @brief サーバーをシャットダウンする
   */
  void shutdown();

  /**
   * @brief クライアントとの接続を処理する
   * @param clientSocket クライアントソケット
   */
  void handleConnection(int clientSocket);

  //  public:
 private:
  INetworkSystem& netSys;                                          // 注入される具象クラスのポインタ
  int listenSocket;                                                // Severのファイルディスクリプタ
  bool isRunning;                                                  // Serverが稼働中ならtrue
  int port;                                                        // サーバーのポート番号
  static constexpr int DEFAULT_BUFLEN = 512;                       // デフォルトのバッファサイズ
  ColorRegionDetectionActionHandler& colorRegionDetectionHandler;  // 色領域検出のハンドラー
};
#endif  // SOCKET_SERVER_H