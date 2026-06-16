/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include <iostream>
#include <cstdio>
#include "EtRobocon2026.h"
#include "RealNetworkSystem.h"
void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient socketClient(real);  // SocketClientインスタンスを生成
  socketClient.connectToServer();
  Robot robot(socketClient);  // RobotインスタンスにSocketClientを渡す

  std::cout << "Hello KATLAB" << std::endl;

  Calibrator calibrator(robot);

  // 復号キーを取得
  calibrator.inputAndSetDecryptionKey();
  const char* decryptionKey = calibrator.getDecryptionKey();
  Logger::printfLog(Logger::INFO, "Retrieved Decryption Key: %s\n", decryptionKey);

  // キャリブレーション等の一連の処理を実行
  calibrator.selectAndSetCourse();
  calibrator.measureAndSetTargetBrightness();
  calibrator.getAngleCheckFrame();
  calibrator.waitForStart();
}
