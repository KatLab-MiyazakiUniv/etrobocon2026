#include "ClockUtil.h"

int ClockUtil::now()
{
  // マイクロ秒をミリ秒になおしてreturn
  return static_cast<int>(getClockInstance().now() / 1000);
}

void ClockUtil::sleep(int milliSec)
{
  // spikeapi::Clock::sleep() はマイクロ秒指定なので、単位を合わせて呼び出す
  getClockInstance().sleep(milliSec * 1000);
}

spikeapi::Clock& ClockUtil::getClockInstance()
{
  // 初回のみインスタンスを生成
  static spikeapi::Clock clock;
  return clock;
}
