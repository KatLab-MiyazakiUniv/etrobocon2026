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
  while(1){
  float az = imu.getAzimuth();
  std::cout << az << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}}
