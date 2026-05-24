#include "SocketServer.h"
#include <iostream>

int main()
{
  SocketServer server;

  if(!server.init()) {
    std::cerr << "サーバーの起動に失敗しました。" << std::endl;
    return 1;
  }

  std::cout << "サーバー稼働開始。クライアントを待機しています..." << std::endl;

  server.run();

  std::cout << "サーバープロセスを安全に終了します。" << std::endl;
  return 0;
}