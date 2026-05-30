/**
 * @file    SocketClient.cpp
 * @brief   カメラサーバーと通信するクラス
 * @author  sadomiya-sousi, okuyama0528
 */

#include "SocketClient.h"

SocketClient::SocketClient(INetworkSystem* networkSystem)
  : netSys(networkSystem), sock(-1), isConnected(false)
{
  std::string msg = "SocketClient: コンストラクタが呼び出されました (netSys: "
                    + std::to_string(reinterpret_cast<uintptr_t>(networkSystem)) + ")";
  Logger::info(msg.c_str());
}

SocketClient::~SocketClient()
{
  std::string msg = "SocketClient: デストラクタが呼び出されました (isConnected: "
                    + std::string(isConnected ? "true" : "false") + ")";
  Logger::info(msg.c_str());
  if(isConnected) {
    Logger::info("SocketClient: まだ接続中のため、自動切断処理へ移行します");
    disconnectFromServer();
  }
}

bool SocketClient::connectToServer(const char* server_ip)
{
  std::string startMsg = "connectToServer: 開始 (引数 server_ip: \""
                         + std::string(server_ip ? server_ip : "NULL") + "\")";
  Logger::info(startMsg.c_str());

  if(isConnected) {
    Logger::info("Already connected.");
    Logger::info("connectToServer: すでに接続済みのため、何もせず終了します (return true)");
    return true;
  }

  Logger::info("connectToServer: socket(AF_INET, SOCK_STREAM, 0) を呼び出します...");
  sock = netSys->socket(AF_INET, SOCK_STREAM, 0);

  std::string sockMsg = "connectToServer: socket 作成結果 (sock = " + std::to_string(sock) + ")";
  Logger::info(sockMsg.c_str());

  if(sock < 0) {
    Logger::error("Client: socket creation failed");
    Logger::info("connectToServer: ソケット作成に失敗しました (return false)");
    return false;
  }

  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);

  std::string portMsg
      = "connectToServer: アドレス構造体を初期化しました (PORT: " + std::to_string(PORT) + ")";
  Logger::info(portMsg.c_str());

  Logger::info("connectToServer: inet_pton を呼び出します...");
  if(inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0) {
    Logger::error("Client: Invalid address/ Address not supported");

    std::string errIpMsg = "connectToServer: IPアドレスの変換に失敗したため、ソケット "
                           + std::to_string(sock) + " を閉じます";
    Logger::info(errIpMsg.c_str());

    netSys->close(sock);
    sock = -1;
    Logger::info("connectToServer: 終了 (return false)");
    return false;
  }

  Logger::info("connectToServer: connect を呼び出してサーバーに接続を試みます...");
  if(netSys->connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    Logger::error("Client: Connection Failed");

    std::string errConnMsg
        = "connectToServer: 接続に失敗したため、ソケット " + std::to_string(sock) + " を閉じます";
    Logger::info(errConnMsg.c_str());

    netSys->close(sock);
    sock = -1;
    Logger::info("connectToServer: 終了 (return false)");
    return false;
  }

  Logger::info("Successfully connected to camera server.");
  Logger::info("connectToServer: 接続成功。ステータスを接続中に更新します");
  isConnected = true;
  Logger::info("connectToServer: 終了 (return true)");
  return true;
}

void SocketClient::disconnectFromServer()
{
  std::string startMsg = "disconnectFromServer: 開始 (現在の状態 - isConnected: "
                         + std::string(isConnected ? "true" : "false")
                         + ", sock: " + std::to_string(sock) + ")";
  Logger::info(startMsg.c_str());

  if(isConnected) {
    CameraServer::Command cmd = CameraServer::Command::DISCONNECT;

    std::string cmdMsg = "disconnectFromServer: DISCONNECT コマンドを送信します (サイズ: "
                         + std::to_string(sizeof(cmd)) + " bytes)...";
    Logger::info(cmdMsg.c_str());

    netSys->send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);

    std::string closeMsg
        = "disconnectFromServer: ソケット " + std::to_string(sock) + " をクローズします...";
    Logger::info(closeMsg.c_str());

    netSys->close(sock);
    sock = -1;
    isConnected = false;
    Logger::info("Disconnected from camera server.");
    Logger::info("disconnectFromServer: クリーンアップ完了");
  } else {
    Logger::info("disconnectFromServer: 未接続状態のため、切断処理をスキップします");
  }
  Logger::info("disconnectFromServer: 終了");
}

void SocketClient::shutdownServer()
{
  std::string startMsg = "shutdownServer: 開始 (現在の状態 - isConnected: "
                         + std::string(isConnected ? "true" : "false")
                         + ", sock: " + std::to_string(sock) + ")";
  Logger::info(startMsg.c_str());

  if(isConnected) {
    CameraServer::Command cmd = CameraServer::Command::SHUTDOWN;

    std::string cmdMsg = "shutdownServer: SHUTDOWN コマンドを送信します (サイズ: "
                         + std::to_string(sizeof(cmd)) + " bytes)...";
    Logger::info(cmdMsg.c_str());

    netSys->send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);

    std::string closeMsg
        = "shutdownServer: ソケット " + std::to_string(sock) + " をクローズします...";
    Logger::info(closeMsg.c_str());

    netSys->close(sock);
    sock = -1;
    isConnected = false;
    Logger::info("Shutdown camera server.");
    Logger::info("shutdownServer: クリーンアップ完了");
  } else {
    Logger::info("shutdownServer: 未接続状態のため、シャットダウンコマンドの送信をスキップします");
  }
  Logger::info("shutdownServer: 終了");
}
