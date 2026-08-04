/**
 * @file   RemoteDecryptService.cpp
 * @brief  復号サーバーへ接続して復号を実行するクラス
 * @author HaruArima08
 */

#include "RemoteDecryptService.h"
#include "Stopwatch.h"
#include <algorithm>
#include <cstring>

RemoteDecryptService::RemoteDecryptService(DecryptClient& _client, DecryptMode _mode)
  : client(_client), mode(_mode)
{
}

bool RemoteDecryptService::decrypt(const std::string& key, const std::string& encryptedText,
                                   uint32_t iterations, DecryptMeasurement& measurement)
{
  measurement = DecryptMeasurement();

  if(key.size() > DecryptServer::MAX_KEY_LENGTH
     || encryptedText.size() > DecryptServer::MAX_ENCRYPTED_TEXT_LENGTH) {
    Logger::error("RemoteDecryptService:decrypt: 復号キーまたは暗号文が長すぎます");
    return false;
  }

  DecryptServer::DecryptRequest request;
  request.keyLength = static_cast<uint8_t>(key.size());
  request.encryptedTextLength = static_cast<uint16_t>(encryptedText.size());
  request.iterations = iterations;
  std::memcpy(request.key, key.data(), key.size());
  std::memcpy(request.encryptedText, encryptedText.data(), encryptedText.size());

  DecryptServer::DecryptResponse response;

  // 送信からレスポンス受信完了までを、呼び出し側から見た全体の時間として計測する
  Stopwatch stopwatch;
  bool isSucceeded = client.executeDecrypt(request, response);
  measurement.totalMicro = stopwatch.elapsedMicro();

  if(!isSucceeded) {
    Logger::error("RemoteDecryptService:decrypt: 復号サーバーとの通信に失敗しました");
    return false;
  }

  measurement.success = response.success;
  measurement.serverDecryptMicro = response.decryptTimeMicro;
  // クロックの精度差で負にならないよう、下限を0に丸める
  measurement.commMicro = measurement.totalMicro > measurement.serverDecryptMicro
                              ? measurement.totalMicro - measurement.serverDecryptMicro
                              : 0;

  size_t plainTextLength = std::min(static_cast<size_t>(response.plainTextLength),
                                    DecryptServer::MAX_PLAIN_TEXT_LENGTH);
  measurement.plainText.assign(response.plainText, plainTextLength);

  return measurement.success;
}

bool RemoteDecryptService::measurePing(uint64_t& roundTripMicro)
{
  DecryptServer::PingRequest request;
  DecryptServer::PingResponse response;

  Stopwatch stopwatch;
  bool isSucceeded = client.executePing(request, response);
  roundTripMicro = stopwatch.elapsedMicro();

  if(!isSucceeded) {
    Logger::error("RemoteDecryptService:measurePing: 復号サーバーとの通信に失敗しました");
    return false;
  }
  return true;
}

DecryptMode RemoteDecryptService::getMode() const
{
  return mode;
}
