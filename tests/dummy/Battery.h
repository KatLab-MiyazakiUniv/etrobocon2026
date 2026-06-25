/**
 * @file Battery.h
 * @brief spikeapi::Battery クラスのモッククラス
 * @author migaku2645
 */

#ifndef MOCK_BATTERY_H
#define MOCK_BATTERY_H

#include <cstdint>

// BatteryController.h が "spikeapi::Clock" を参照しているため、名前空間を合わせる
namespace spikeapi {
  class Battery {
   public:
    /**
     * コンストラクタ
     * @param -
     * @return -
     */
    Battery(void) = default;

    /**
     * 電流取得
     * バッテリーの電流を取得する
     * @param -
     * @return 電流[mA]
     */
    uint16_t getCurrent(void) const { return current; }

    /**
     * 電圧取得
     * バッテリーの電圧を取得する
     * @param -
     * @return 電圧[mV]
     */
    uint16_t getVoltage(void) const { return voltage; }

   private:
    uint16_t voltage = 100;
    uint16_t current = 200;

  };  // class Battery

}  // namespace spikeapi

#endif