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
   * @brief 開始時間からの経過時間取得
   * @return 経過時間(ミリ秒)
   */
  static int now();

  /**
   * @brief 自タスクススリープ（デフォルトは10ミリ秒）
   * @param milliSec スリープ時間(ミリ秒)
   */
  static void sleep(int milliSec = 10);

  /**
   * @brief 自タスク遅延（デフォルトは10ミリ秒）
   * @param milliSec 遅延時間(ミリ秒)
   */
  static void wait(int milliSec = 10);

 private:
  ClockUtil();  // インスタンス化を禁止する

#ifdef USE_RASPIKE_ART
  /**
   * @brief spikeapi::Clock のインスタンスの参照を返す（初回のみ生成を行う）
   * @return spikeapi::Clock のインスタンスの参照
   */
  static spikeapi::Clock& getClockInstance();
#else
  /**
   * @brief custom::Clock のインスタンスの参照を返す（初回のみ生成を行う）
   * @return custom::Clock のインスタンスの参照
   */
  static custom::Clock& getClockInstance();
#endif
};

#endif