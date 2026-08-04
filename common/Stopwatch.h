/**
 * @file   Stopwatch.h
 * @brief  マイクロ秒精度で経過時間を計測するクラス
 * @author HaruArima08
 */

#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <cstdint>
#include "ClockUtil.h"

class Stopwatch {
 public:
  /**
   * @brief コンストラクタ（生成した時点で計測を開始する）
   */
  Stopwatch() : startMicro(ClockUtil::nowMicro()) {}

  /**
   * @brief 計測開始時刻を現在時刻に更新する
   */
  void reset() { startMicro = ClockUtil::nowMicro(); }

  /**
   * @brief 計測開始からの経過時間を取得する
   * @return 経過時間(マイクロ秒)
   */
  uint64_t elapsedMicro() const { return ClockUtil::nowMicro() - startMicro; }

 private:
  uint64_t startMicro;  // 計測開始時刻(マイクロ秒)
};

#endif  // STOPWATCH_H
