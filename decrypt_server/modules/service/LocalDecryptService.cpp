/**
 * @file   LocalDecryptService.cpp
 * @brief  自プロセス内で復号を実行するクラス
 * @author HaruArima08
 */

#include "LocalDecryptService.h"
#include "Decrypter.h"
#include "Stopwatch.h"
#include "Logger.h"
#include <algorithm>

LocalDecryptService::LocalDecryptService()
{
  LOG_CREATE("LocalDecryptService");
}

bool LocalDecryptService::decrypt(const std::string& key, const std::string& encryptedText,
                                  uint32_t iterations, DecryptMeasurement& measurement)
{
  measurement = DecryptMeasurement();

  uint32_t decryptCount = std::max(iterations, 1u);
  std::string plainText;

  Stopwatch stopwatch;
  for(uint32_t i = 0; i < decryptCount; i++) {
    plainText = Decrypter::decrypt(key, encryptedText);
  }
  measurement.totalMicro = stopwatch.elapsedMicro();

  // 通信を行わないため、全体の時間がそのまま演算時間になる
  measurement.serverDecryptMicro = measurement.totalMicro;
  measurement.commMicro = 0;
  measurement.plainText = plainText;
  measurement.success = !plainText.empty();

  if(!measurement.success) {
    Logger::error("LocalDecryptService:decrypt: 復号に失敗しました");
  }
  return measurement.success;
}

bool LocalDecryptService::measurePing(uint64_t& roundTripMicro)
{
  // 通信を行わないモードなので往復時間は常に0とする
  roundTripMicro = 0;
  return true;
}

DecryptMode LocalDecryptService::getMode() const
{
  return DecryptMode::LOCAL_INPROC;
}
