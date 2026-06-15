
/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "RealNetworkSystem.h"
void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient socketClient(real);  // SocketClientインスタンスを生成
  Robot robot(socketClient);        // RobotインスタンスにSocketClientを渡す

  std::cout << "Hello KATLAB" << std::endl;
  // robot.getIMUControllerInstance().initializeOffset();
  // robot.getIMUControllerInstance().calculateCorrectionMatrix();

  // if(robot.getSocketClient().connectToServer()) {
  //   std::cout << "Connected to server." << std::endl;
  // } else {
  //   std::cout << "Failed to connect to server." << std::endl;
  //   return;
  // };

  // Calibrator calibrator(robot);
  // calibrator.selectAndSetCourse();
  // calibrator.measureAndSetTargetBrightness();
  // bool isLeftCourse = calibrator.getIsLeftCourse();
  // int targetBrightness = calibrator.getTargetBrightness();
  // // calibrator.getAngleCheckFrame();
  // calibrator.waitForStart();
}
