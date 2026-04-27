/**
 * @file app.cpp
 * @brief タスクを管理するクラス
 * @author HaruArima08
 */

#include "app.h"
#include "EtRobocon2026.h"

// メインタスク
void main_task(intptr_t unused)
{
  printf("Hello ETROBO!\n");
  EtRobocon2026::start();
  ext_tsk();
}