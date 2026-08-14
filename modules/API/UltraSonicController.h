/**
 * @file UltraSonicController.h
 * @brief 超音波センサーを操作するクラス
 * @author nishijima515
 */

#ifndef ULTRA_SONIC_CONTROLLER_H
#define ULTRA_SONIC_CONTROLLER_H

#include "UltrasonicSensor.h"

class UltraSonicController {
 public:
  /**
   * @brief コンストラクタ
   */
  UltraSonicController();

  /**
   * @brief 超音波センサーとの距離を取得する
   * @return 超音波センサーとの距離（mm）
   */
  int getDistance();

 private:
  spikeapi::UltrasonicSensor ultrasonicSensor;  // 超音波センサーのインスタンス
};

#endif  // ULTRA_SONIC_CONTROLLER_H