/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "IMUController.h"
#include <thread>
#include <chrono>

void EtRobocon2026::start()
{
  std::cout << "Hello KATLAB" << std::endl;
  IMUController imu;
  float accel[3] = { 0, 0, 0 };
  float ang[3] = { 0, 0, 0 };
  bool isReady = imu.isReady();
  bool isStationary;
  std::cout << "isReady" << std::endl;
  std::cout << isReady << std::endl;
  std::cout << "getAzimuth" << std::endl;
  for(int i = 0; i <= 5; i++) {
    float az = imu.getAzimuth();
    std::cout << az << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  std::cout << "getAcceleration" << std::endl;
  for(int i = 0; i <= 5; i++) {
    imu.getAcceleration(accel);
    std::cout << accel << std::endl;
  }
  std::cout << "getAngularBelocity" << std::endl;
  for(int i = 0; i <= 5; i++) {
    imu.getAngularVelocity(ang);
    std::cout << ang << std::endl;
  }
  std::cout << "isStationary" << std::endl;
  for(int i = 0; i <= 5; i++) {
    bool isStationary = imu.isStationary();
    std::cout << isStationary << std::endl;
  }
  imu.setTilt(90);
  std::cout << "resetAzimuth" << std::endl;
  imu.resetAzimuth();
  float az = imu.getAzimuth();
  std::cout << az << std::endl;
}
