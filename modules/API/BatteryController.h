/**
 * @file BatteryController.h
 * @brief バッテリーの電流と電圧を取得するクラス
 * @author migaku2645
 */
#ifndef BATTERY_CONTROLLER_H
#define BATTERY_CONTROLLER_H

#include <cstdint>
#include "Battery.h"

class BatteryController {
 public:
  /**
   * 電流取得
   * バッテリーの電流を取得する
   * @return 電流[mA]
   */
  static uint16_t getCurrent();

  /**
   * 電圧取得
   * バッテリーの電圧を取得する
   * @return 電圧[mV]
   */
  static uint16_t getVoltage();

 private:
  BatteryController();
  static spikeapi::Battery& getBatteryInstance();
};

#endif