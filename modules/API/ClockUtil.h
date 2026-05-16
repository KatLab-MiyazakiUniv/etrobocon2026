/**
 * @file ClockUtil.h
 * @brief クロック操作に用いる関数をまとめたユーティリティクラス
 * @author migaku2645
 */
#ifndef CLOCK_UTIL_H
#define CLOCK_UTIL_H

#include "Clock.h"

class ClockUtil {
 public:
  /**
   * @brief 走行時間を取得
   * @return 走行時間(ミリ秒)
   */
  static int now();

  /**
   * @brief 自タスクスリープ（デフォルトは10ミリ秒）
   * @param milliSec スリープ時間(ミリ秒)
   */
  static void sleep(int milliSec = 10);

  /**
   * @brief 自タスクウェイト（デフォルトは10ミリ秒）
   * @param milliSec ウェイト時間(ミリ秒)
   */
  static void wait(int milliSec = 10);

 private:
  ClockUtil();  // インスタンス化を禁止する

  /**
   * @brief spikeapi::Clock のインスタンスの参照を返す（初回のみ生成を行う）
   * @return spikeapi::Clock のインスタンスの参照
   */
  static spikeapi::Clock& getClockInstance();
};

#endif