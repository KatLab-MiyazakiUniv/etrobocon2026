#ifndef MOCK_CLOCK_H
#define MOCK_CLOCK_H

// ClockUtil.h が "spikeapi::Clock" を参照しているため、名前空間を合わせます
namespace spikeapi {
  class Clock {
   public:
    Clock() {}

    // ClockUtil が使用する可能性のあるメソッドを定義
    void sleep(unsigned int duration) {}
    void wait(unsigned int duration) {}
    void reset() {}
    unsigned int now() { return 0; }
  };
}  // namespace spikeapi

#endif