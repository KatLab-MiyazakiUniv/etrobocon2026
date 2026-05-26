/**
 * @file Clock.h
 * @brief クロック関数の自前クラス
 * @author miyahara046
 */
#ifndef CLOCK_H
#define CLOCK_H

#include <chrono>
#include <cstdint>
#include <thread>

namespace custom {

class Clock {
public:
/**
 * @brief コンストラクタ
 */  
Clock()
    : startTime(std::chrono::steady_clock::now())
  {
  }
  
  /**
   * @brief 現在時刻をマイクロ秒で取得
   */
  uint64_t now() const
  {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now() - startTime)
        .count();
  }

  /**
   * @brief 指定時間スリープ
   * @param microSec マイクロ秒
   */
  void sleep(uint64_t microSec) const
  {
    std::this_thread::sleep_for(std::chrono::microseconds(microSec));
  }

  /**
   * @brief wait互換
   * @param microSec マイクロ秒
   */
  void wait(uint64_t microSec) const
  {
    std::this_thread::sleep_for(std::chrono::microseconds(microSec));
  }

private:
    std::chrono::steady_clock::time_point startTime;  // クロックの開始時間

};

} // namespace custom
#endif