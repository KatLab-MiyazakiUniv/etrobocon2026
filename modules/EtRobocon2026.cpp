/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include <unistd.h>
#include <cstdint>  // int32_tなどの型を使うため
#include "ColorSensorController.h"

void EtRobocon2026::start()
{
  ColorSensorController colorSensorController;
  std::cout << "Hello KATLAB" << std::endl;

  // ColorSensorController::RGB rgb;
  // ColorSensorController::HSV hsv;
  int reflection;

  while(true) {
    reflection = colorSensorController.getReflectance();
    std::cout << "周囲の明るさの強度は" << reflection << std::endl;
    sleep(1);
  }
}