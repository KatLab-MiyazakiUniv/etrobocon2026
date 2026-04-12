/**
 * @file app.cpp
 * @brief タスクを管理するクラス
 * @author HaruArima08
 */

#include "app.h"
#include <stdio.h>
#include "EtRobocon2026.h"

// メインタスク
void main_task(intptr_t unused)
{
  std::cout << "Hello ETROBO!" << std::endl;
  EtRobocon2026::start();
  ext_tsk();
}