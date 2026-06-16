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
  int inputVal;
  Logger::info("標準入力ができるか検証\n");
  std::cin >> inputVal;
  Logger::printfLog(Logger::INFO, "cin >> inputVal succeeded:%d\n", inputVal);
}
