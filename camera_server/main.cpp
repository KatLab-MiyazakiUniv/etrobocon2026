#include "SocketServer.h"
#include <iostream>

int main()
{
  SocketServer server;

  // 1. サーバーの初期化（ポートの開放など）
  if(!server.init()) {
    std::cerr << "サーバーの起動に失敗しました。" << std::endl;
    return 1;
  }

  std::cout << "サーバー稼働開始。クライアントを待機しています..." << std::endl;

  // 2. 接続待ちループの開始（ここでプログラムがブロック・待機します）
  // クライアントからSHUTDOWNが送られてくると、ループを抜けて終了します。
  server.run();

  std::cout << "サーバープロセスを安全に終了します。" << std::endl;
  return 0;
}