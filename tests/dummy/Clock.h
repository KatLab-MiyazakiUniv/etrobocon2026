/**
 * @file Clock.h
 * @brief spikeapi::Clock クラスのモッククラス
 * @author migaku2645
 */

#ifndef MOCK_CLOCK_H
#define MOCK_CLOCK_H

#include <cstdint>

// ClockUtil.h が "spikeapi::Clock" を参照しているため、名前空間を合わせます
namespace spikeapi {
  class Clock {
   public:
    Clock() { microTime = 0; }

    // ClockUtil が使用する可能性のあるメソッドを定義
    uint64_t now() { return microTime += 1000; }
    void sleep(unsigned int duration) { microTime += static_cast<uint64_t>(duration); }
    void wait(unsigned int duration) { microTime += static_cast<uint64_t>(duration); }

   private:
    uint64_t microTime;
  };

}  // namespace spikeapi

#endif