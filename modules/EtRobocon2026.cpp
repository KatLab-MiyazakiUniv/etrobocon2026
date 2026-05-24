
#include <iostream>
#include <unistd.h>  // sleep関数用

#include "EtRobocon2026.h"
#include "SocketClient.h"

void EtRobocon2026::start()
{
  SocketClient client;

  std::cout << "カメラサーバーに接続を試みます..." << std::endl;

  // 1. サーバーへ接続
  if(client.connectToServer()) {
    std::cout << "接続成功！3秒後にシャットダウンコマンドを送信します..." << std::endl;

    // 通信が確立したことを確認するため、あえて3秒待ちます
    sleep(3);

    // 2. 切断処理（内部で自動的に SHUTDOWN コマンドが送信されます）
    client.disconnectFromServer();
  } else {
    std::cerr << "サーバーが見つかりません。サーバーが起動しているか確認してください。"
              << std::endl;
    return 1;
  }

  return 0;
}
