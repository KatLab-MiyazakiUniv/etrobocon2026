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
  //  Logger::printfLog(Logger::INFO, "(netSys: %p, port: %d, server_ip: %s)", netSys, port,
  //  server_ip);
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
    Logger::info("接続済み");
    return true;
  }

  Logger::info("connectToServer: socket()実行前");
  // sock = netSys->socket(AF_INET, SOCK_STREAM, 0);
  sock = netSys.socket(AF_INET, SOCK_STREAM, 0);

  Logger::printfLog(Logger::INFO, "connectToServer: socket()の実行後 %d", sock);
  if(sock < 0) {
    Logger::info("connectToServer: socket()失敗");
    return false;
  }

  Logger::info("connectToServer: アドレス構造体初期化前");
  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(this->port);
  Logger::info("connectToServer: inet_pton()実行前");
  if(inet_pton(AF_INET, serverIp.c_str(), &serv_addr.sin_addr) <= 0) {
    Logger::printfLog(Logger::INFO, "connectToServer: inet_pton()失敗:ソケット %d ", sock);
    // close()が成功するか
    // if(netSys->close(sock) < 0) {
    if(netSys.close(sock) < 0) {
      Logger::error("init: close()失敗");
    }
    sock = -1;
    Logger::info("connectToServer: 失敗.処理終了");
    return false;
  }
  Logger::printfLog(Logger::INFO, "connectToServer: アドレス構造体初期化後 (PORT: %d)", this->port);

  Logger::info("connectToServer: connect()実行前");
  // if(netSys->connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
  if(netSys.connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    Logger::printfLog(Logger::ERROR, "connectToServer: connect():ソケット %d を閉じます", sock);
    // close()が成功するか
    // if(netSys->close(sock) < 0) {
    if(netSys.close(sock) < 0) {
      Logger::error("init: close()失敗");
    }

    sock = -1;
    Logger::error("connectToServer:connect()失敗");
    return false;
  }

  Logger::info("connectToServer: 接続成功");
  isConnected = true;
  return true;
}

void SocketClient::disconnectFromServer()
{
  Logger::printfLog(Logger::INFO, "disconnectFromServer:開始");
  if(isConnected) {
    CameraServer::Command cmd = CameraServer::Command::DISCONNECT;

    Logger::printfLog(Logger::INFO, "disconnectFromServer: DISCONNECT コマンドを送信 ");
    // netSys->send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);
    netSys.send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);

    Logger::printfLog(Logger::INFO, "disconnectFromServer: ソケット %d をクローズします...", sock);
    // close()が成功するか
    // if(netSys->close(sock) < 0) {
    if(netSys.close(sock) < 0) {
      Logger::error("init: close()失敗");
    }
    sock = -1;
    isConnected = false;
    Logger::info("Disconnected from camera server.");
  } else {
    Logger::info("disconnectFromServer: 未接続状態のためスキップ");
  }
  Logger::info("disconnectFromServer: 終了");
}

void SocketClient::shutdownServer()
{
  Logger::printfLog(Logger::INFO, "shutdownServer: 開始");
  Logger::printfLog(Logger::INFO, "shutdown:isConnected: %s, sock: %d)",
                    isConnected ? "true" : "false", sock);
  if(isConnected) {
    CameraServer::Command cmd = CameraServer::Command::SHUTDOWN;

    Logger::printfLog(Logger::INFO, "shutdownServer: SHUTDOWN コマンドを送信前");
    // netSys->send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);
    netSys.send(sock, reinterpret_cast<const char*>(&cmd), sizeof(cmd), 0);

    Logger::printfLog(Logger::INFO, "shutdownServer: close()前 ソケット :%d ", sock);
    // close()が成功するか
    // if(netSys->close(sock) < 0) {
    if(netSys.close(sock) < 0) {
      Logger::error("init: close()失敗");
    }

    sock = -1;
    isConnected = false;
    Logger::info("Shutdown camera server.");
  } else {
    Logger::info("shutdownServer: 未接続状態のためスキップ");
  }
  Logger::info("shutdownServer: 終了");
}
