/**
 * @file   EtRobocon2026.h
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include <iostream>
#include <unistd.h>

#include "EtRobocon2026.h"
#include "SocketClient.h"

void EtRobocon2026::start()
{
  SocketClient client;
  std::cout << "Hello KATLAB" << std::endl;
  std::cout << "カメラサーバーに接続を試みます..." << std::endl;
  if(client.connectToServer()) {
    std::cout << "接続成功！3秒後にシャットダウンコマンドを送信します..." << std::endl;
    sleep(3);
    client.disconnectFromServer();

  } else {
    std::cerr << "サーバーが見つかりません。サーバーが起動しているか確認してください。"
              << std::endl;
  }
}
