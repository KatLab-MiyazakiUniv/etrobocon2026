/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "Logger.h"
#include "Robot.h"
#include "Calibrator.h"
#include <iostream>
#include <cstdio>
#include "EtRobocon2026.h"
#include "RealNetworkSystem.h"
void EtRobocon2026::start()

{
  Logger::info("Hello KATLAB");
  std::string test;

  std::cout << "Enter test:" << std::endl;
  std::cin >> test;
  std::cout << "test=" << test << std::endl;

  RealNetworkSystem real;
  SocketClient socketClient(real);
  socketClient.connectToServer();

  Robot robot(socketClient);

  std::cout << "Hello KATLAB" << std::endl;

  Calibrator calibrator(robot);

  // 復号キー入力（Robotに保存される）
  calibrator.inputAndSetDecryptionKey();

  // :white_check_mark: Robotから取得する
  const char* decryptionKey = robot.getDecryptionKey();
  Logger::printfLog(Logger::INFO, "Retrieved Decryption Key: %s", decryptionKey);

  // キャリブレーション
  calibrator.selectAndSetCourse();
  calibrator.measureAndSetTargetBrightness();
  calibrator.getAngleCheckFrame();
  calibrator.waitForStart();
}
