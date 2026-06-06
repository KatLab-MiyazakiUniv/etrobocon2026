/**
 * @file    SocketClient.cpp
 * @brief   カメラサーバーと通信するクラス
 * @author  sadomiya-sousi, okuyama0528
 */

#include "SocketClient.h"

SocketClient::SocketClient(INetworkSystem& _netSys, int _port, const char* _serverIp)
  : netSys(_netSys), sock(-1), isConnected(false), port(_port), serverIp(_serverIp)
{
  LOG_CREATE("SocketClient");
  // 初期化リストで,代入してるのに抽象クラスの出力と判定されerrorに..
  // Logger::printfLog(Logger::INFO, "(netSys: %p, port: %d, serverIp: %s)", netSys, port,
  // serverIp);
}

SocketClient::~SocketClient()
{
  LOG_DESTROY("SocketClient");
  if(isConnected) {
    disconnectFromServer();
  }
}

bool SocketClient::connectToServer()
{
  Logger::printfLog(Logger::INFO, "connectToServer: 開始 (IP: %s, Port: %d)", serverIp, port);

  if(isConnected) {
    Logger::debug("接続済み");
    return true;
  }

  Logger::debug("connectToServer: socket()実行前");
  sock = netSys.socket(AF_INET, SOCK_STREAM, 0);

  Logger::printfLog(Logger::DEBUG, "connectToServer: socket()の実行後 %d", sock);
  if(sock < 0) {
    Logger::debug("connectToServer: socket()失敗");
    return false;
  }

  Logger::debug("connectToServer: アドレス構造体初期化前");
  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(this->port);
  Logger::info("connectToServer: inet_pton()実行前");
  if(inet_pton(AF_INET, serverIp.c_str(), &serv_addr.sin_addr) <= 0) {
    Logger::printfLog(Logger::DEBUG, "connectToServer: inet_pton()失敗:ソケット %d ", sock);
    netSys.close(sock);
    sock = -1;
    Logger::debug("connectToServer: 失敗.処理終了");
    return false;
  }
  Logger::printfLog(Logger::DEBUG, "connectToServer: アドレス構造体初期化後 (PORT: %d)",
                    this->port);

  Logger::debug("connectToServer: connect()実行前");
  if(netSys.connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    Logger::printfLog(Logger::ERROR, "connectToServer: connect():ソケット %d を閉じます", sock);
    netSys.close(sock);
    sock = -1;
    Logger::error("connectToServer:connect()失敗");
    return false;
  }

  Logger::debug("connectToServer: 接続成功");
  isConnected = true;
  return true;
}

void SocketClient::disconnectFromServer()
{
  Logger::printfLog(Logger::DEBUG, "disconnectFromServer:開始");
  if(isConnected) {
    CameraServer::Command cmd = CameraServer::Command::DISCONNECT;

    Logger::printfLog(Logger::DEBUG, "disconnectFromServer: DISCONNECT コマンドを送信 ");
    netSys.send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);

    Logger::printfLog(Logger::DEBUG, "disconnectFromServer: ソケット %d をクローズします...", sock);
    netSys.close(sock);
    sock = -1;
    isConnected = false;
    Logger::debug("Disconnected from camera server.");
  } else {
    Logger::debug("disconnectFromServer: 未接続状態のためスキップ");
  }
  Logger::debug("disconnectFromServer: 終了");
}

void SocketClient::shutdownServer()
{
  Logger::printfLog(Logger::DEBUG, "shutdownServer: 開始");
  Logger::printfLog(Logger::DEBUG, "shutdown:isConnected: %s, sock: %d)",
                    isConnected ? "true" : "false", sock);
  if(isConnected) {
    CameraServer::Command cmd = CameraServer::Command::SHUTDOWN;

    Logger::printfLog(Logger::DEBUG, "shutdownServer: SHUTDOWN コマンドを送信前");
    netSys.send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);

    Logger::printfLog(Logger::DEBUG, "shutdownServer: close()前 ソケット :%d ", sock);
    netSys.close(sock);

    sock = -1;
    isConnected = false;
    Logger::debug("Shutdown camera server.");
  } else {
    Logger::debug("shutdownServer: 未接続状態のためスキップ");
  }
  Logger::debug("shutdownServer: 終了");
}
