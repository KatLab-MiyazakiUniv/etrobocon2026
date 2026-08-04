/**
 * @file   DecryptClient.h
 * @brief  復号サーバーと通信するクラス
 * @author HaruArima08
 */

#ifndef DECRYPT_CLIENT_H
#define DECRYPT_CLIENT_H

#include "RealNetworkSystem.h"
#include "DecryptProtocol.h"
#include "SocketIo.h"
#include "Logger.h"
#include <arpa/inet.h>
#include <string.h>
#include <string>

class DecryptClient {
 public:
  /**
   * @brief コンストラクタ
   * @param _netSys 注入する具象クラス
   * @param _port デフォルトは27016
   * @param _serverIp デフォルトはローカルアドレス
   */
  explicit DecryptClient(INetworkSystem& _netSys, int _port = DecryptServer::DEFAULT_PORT,
                         const char* _serverIp = DecryptServer::LOCAL_SERVER_IP);

  /**
   * @brief デストラクタ
   */
  virtual ~DecryptClient();

  /**
   * @brief サーバーに接続する
   * @return true 接続に成功した場合
   * @return false 接続に失敗した場合
   */
  bool connectToServer();

  /**
   * @brief サーバーから切断する
   */
  void disconnectFromServer();

  /**
   * @brief サーバーをシャットダウンする
   */
  void shutdownServer();

  /**
   * @brief 復号アクションを実行する
   * @param request リクエスト
   * @param response レスポンス
   * @return true アクションの実行に成功した場合
   * @return false アクションの実行に失敗した場合
   */
  virtual bool executeDecrypt(const DecryptServer::DecryptRequest& request,
                              DecryptServer::DecryptResponse& response);

  /**
   * @brief Pingアクションを実行する
   * @param request リクエスト
   * @param response レスポンス
   * @return true アクションの実行に成功した場合
   * @return false アクションの実行に失敗した場合
   */
  virtual bool executePing(const DecryptServer::PingRequest& request,
                           DecryptServer::PingResponse& response);

  /**
   * @brief ソケットファイルディスクリプタを取得する
   * @return int ソケットファイルディスクリプタ
   */
  int getSock() const;

  /**
   * @brief サーバーへの接続状態を取得する
   * @return true 接続されている場合
   * @return false 接続されていない場合
   */
  bool getIsConnected() const;

  /**
   * @brief サーバーのポート番号を取得する
   * @return int ポート番号
   */
  int getPort() const;

  /**
   * @brief サーバーのポート番号を設定する
   * @param _port 設定するポート番号
   */
  void setPort(int _port);

  /**
   * @brief サーバーのIPアドレスを取得する
   * @return const std::string& サーバーのIPアドレスへの参照
   */
  const std::string& getServerIp() const;

  /**
   * @brief サーバーのIPアドレスを設定する
   * @param _serverIp 設定するIPアドレス
   */
  void setServerIp(const std::string& _serverIp);

 private:
  INetworkSystem& netSys;  // 注入される具象クラスへの参照
  int sock;                // ソケットファイルディスクリプタ(セッター無し)
  bool isConnected;        // サーバーへの接続状態(セッター無し)
  int port;                // サーバーのポート番号
  std::string serverIp;    // サーバーのIPアドレス

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
  bool executeAction(const Req& request, Res& response)
  {
    if(!isConnected) {
      Logger::error("DecryptClient:executeAction: サーバーに未接続");
      return false;
    }

    // 送受信は分割される可能性があるため、必ず全長を送受信しきる
    if(!SocketIo::sendAll(netSys, sock, &request, sizeof(request))) {
      Logger::error("DecryptClient:executeAction: リクエストの送信に失敗しました");
      return false;
    }

    if(!SocketIo::recvExact(netSys, sock, &response, sizeof(response))) {
      Logger::error("DecryptClient:executeAction: レスポンスの受信に失敗しました");
      return false;
    }
    return true;
  }
};

#endif  // DECRYPT_CLIENT_H
