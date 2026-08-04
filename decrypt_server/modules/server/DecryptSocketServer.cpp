/**
 * @file   DecryptSocketServer.cpp
 * @brief  接続を待ち、クライアントからの復号リクエストを処理するクラス
 * @author HaruArima08
 */

#include "DecryptSocketServer.h"

DecryptSocketServer::DecryptSocketServer(DecryptActionHandler& _decryptHandler,
                                         INetworkSystem& _netSys, int _port)
  : netSys(_netSys),
    listenSocket(-1),
    isRunning(false),
    port(_port),
    decryptHandler(_decryptHandler)
{
  LOG_CREATE("DecryptSocketServer");
  Logger::printfLog(Logger::INFO, "DecryptSocketServer:ポート番号は%d", _port);
}

DecryptSocketServer::~DecryptSocketServer()
{
  LOG_DESTROY("DecryptSocketServer");
  shutdown();
}

bool DecryptSocketServer::init()
{
  listenSocket = netSys.socket(AF_INET, SOCK_STREAM, 0);
  if(listenSocket < 0) {
    Logger::error("DecryptSocketServer:init: socket()失敗");
    return false;
  }

  int opt = 1;
  if(netSys.setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    Logger::error("DecryptSocketServer:init: setsockopt()失敗");
    netSys.close(listenSocket);
    listenSocket = -1;
    return false;
  }

  struct sockaddr_in servAddr;
  memset(&servAddr, 0, sizeof(servAddr));
  servAddr.sin_family = AF_INET;
  servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servAddr.sin_port = htons(this->port);

  if(netSys.bind(listenSocket, (struct sockaddr*)&servAddr, sizeof(servAddr)) < 0) {
    Logger::error("DecryptSocketServer:init: bind()失敗");
    netSys.close(listenSocket);
    listenSocket = -1;
    return false;
  }

  if(netSys.listen(listenSocket, LISTEN_BACKLOG) < 0) {
    Logger::error("DecryptSocketServer:init: listen()失敗");
    netSys.close(listenSocket);
    listenSocket = -1;
    return false;
  }

  Logger::info("DecryptSocketServer:init: 起動成功");
  isRunning = true;
  return true;
}

void DecryptSocketServer::run()
{
  while(isRunning) {
    int clientSocket = netSys.accept(listenSocket, (struct sockaddr*)NULL, NULL);
    if(clientSocket < 0) {
      Logger::error("DecryptSocketServer:run: accept()失敗");
      if(!isRunning) break;
      continue;
    }
    handleConnection(clientSocket);
    netSys.close(clientSocket);
  }
}

void DecryptSocketServer::shutdown()
{
  isRunning = false;
  if(listenSocket != -1) {
    netSys.close(listenSocket);
    listenSocket = -1;
  }
  Logger::info("DecryptSocketServer:shutdown: 復号サーバーをシャットダウンします");
}

void DecryptSocketServer::handleConnection(int clientSocket)
{
  // 1接続内で複数のリクエストを処理する（計測時に接続確立の時間が毎回上乗せされるのを避けるため）
  while(true) {
    DecryptServer::Command command = DecryptServer::Command::DISCONNECT;
    // 可変長のリクエストを判別するため、まず先頭のコマンド1バイトだけを受信する
    if(!SocketIo::recvExact(netSys, clientSocket, &command, DecryptServer::COMMAND_SIZE)) return;

    switch(command) {
      case DecryptServer::Command::DECRYPT:
        if(!handleDecryptCommand(clientSocket)) return;
        break;

      case DecryptServer::Command::PING:
        if(!handlePingCommand(clientSocket)) return;
        break;

      case DecryptServer::Command::SHUTDOWN:
        shutdown();
        return;

      case DecryptServer::Command::DISCONNECT:
        // クライアントからの切断要求なのでreturn
        return;

      default:
        Logger::printfLog(Logger::ERROR,
                          "DecryptSocketServer:handleConnection: 未知のコマンドを受信しました: %u",
                          static_cast<unsigned int>(command));
        return;
    }
  }
}

bool DecryptSocketServer::handleDecryptCommand(int clientSocket)
{
  DecryptServer::DecryptRequest request;
  // コマンド1バイトは受信済みなので、2バイト目以降を構造体の続きへ直接読み込む
  char* body = reinterpret_cast<char*>(&request) + DecryptServer::COMMAND_SIZE;
  size_t bodySize = sizeof(request) - DecryptServer::COMMAND_SIZE;
  if(!SocketIo::recvExact(netSys, clientSocket, body, bodySize)) {
    Logger::error("DecryptSocketServer:handleDecryptCommand: リクエストの受信に失敗しました");
    return false;
  }
  request.command = DecryptServer::Command::DECRYPT;

  DecryptServer::DecryptResponse response;
  decryptHandler.execute(request, response);

  if(!SocketIo::sendAll(netSys, clientSocket, &response, sizeof(response))) {
    Logger::error("DecryptSocketServer:handleDecryptCommand: レスポンスの送信に失敗しました");
    return false;
  }
  return true;
}

bool DecryptSocketServer::handlePingCommand(int clientSocket)
{
  DecryptServer::PingRequest request;
  char* body = reinterpret_cast<char*>(&request) + DecryptServer::COMMAND_SIZE;
  size_t bodySize = sizeof(request) - DecryptServer::COMMAND_SIZE;
  if(!SocketIo::recvExact(netSys, clientSocket, body, bodySize)) {
    Logger::error("DecryptSocketServer:handlePingCommand: リクエストの受信に失敗しました");
    return false;
  }

  DecryptServer::PingResponse response;
  if(!SocketIo::sendAll(netSys, clientSocket, &response, sizeof(response))) {
    Logger::error("DecryptSocketServer:handlePingCommand: レスポンスの送信に失敗しました");
    return false;
  }
  return true;
}

int DecryptSocketServer::getListenSocket() const
{
  return listenSocket;
}

bool DecryptSocketServer::getIsRunning() const
{
  return isRunning;
}

int DecryptSocketServer::getPort() const
{
  return port;
}

void DecryptSocketServer::setPort(int _port)
{
  port = _port;
}

INetworkSystem& DecryptSocketServer::getNetSys() const
{
  return netSys;
}
