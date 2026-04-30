/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "IMUController.h"

void EtRobocon2026::start()
{
  std::cout << "Hello KATLAB" << std::endl;
  IMUController imu;
  float az = imu.getAzimuth();
  std::cout << az << std::endl;
}
