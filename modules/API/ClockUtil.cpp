/**
 * @file ClockUtil.cpp
 * @brief クロック操作に用いる関数をまとめたユーティリティクラス
 * @author migaku2645
 */

#include "ClockUtil.h"
#include <iostream>

double ClockUtil::now()
{
  // マイクロ秒を秒になおしてreturn
  return (getClockInstance().now() / 1000000);
}

void ClockUtil::sleep(int milliSec)
{
  if(milliSec <= 0) {
    std::cerr << "ClockUtil::sleep() was called with non-positive milliSec: " << milliSec
              << std::endl;
    milliSec = 0;  // 負の値は0に補正
  }
  // spikeapi::Clock::sleep() はマイクロ秒指定なので、単位を合わせて呼び出す
  getClockInstance().sleep(milliSec * 1000);
}

void ClockUtil::wait(int milliSec)
{
  if(milliSec <= 0) {
    std::cerr << "ClockUtil::wait() was called with non-positive milliSec: " << milliSec
              << std::endl;
    milliSec = 0;  // 負の値は0に補正
  }
  getClockInstance().sleep(milliSec * 1000);
}

spikeapi::Clock& ClockUtil::getClockInstance()
{
  // 初回のみインスタンスを生成
  static spikeapi::Clock clock;
  return clock;
}
