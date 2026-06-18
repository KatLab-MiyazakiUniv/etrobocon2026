/**
 * @file UltrasonicSensor.h
 * @brief 超音波センサークラス(ダミー)
 * @author nishijima515
 */

#ifndef SPIKE_CPP_API_ULTRASONIC_SENSOR_H_
#define SPIKE_CPP_API_ULTRASONIC_SENSOR_H_

#include <cstdint>
#include "Port.h"
#include "spikeapi.h"

namespace spikeapi {

  class UltrasonicSensor {
   public:
    /**
     * @brief コンストラクタ
     * @param port 使用するポート
     */
    explicit UltrasonicSensor(EPort port) { (void)port; }

    /**
     * @brief 距離を取得する
     * @return 距離 (mm)
     */
    int32_t getDistance() const { return DISTANCE; }

   private:
    static constexpr int32_t DISTANCE = 100;  // テスト用固定値 (mm)
  };

}  // namespace spikeapi

#endif  // SPIKE_CPP_API_ULTRASONIC_SENSOR_H_
