/**
 * @file   DecryptActionHandler.cpp
 * @brief  復号を実行し演算時間を計測するクラス
 * @author HaruArima08
 */

#include "DecryptActionHandler.h"
#include "Stopwatch.h"
#include <algorithm>
#include <cstring>
#include <string>

DecryptActionHandler::DecryptActionHandler()
{
  LOG_CREATE("DecryptActionHandler");
}

void DecryptActionHandler::execute(const DecryptServer::DecryptRequest& request,
                                   DecryptServer::DecryptResponse& response)
{
  // 不正な長さを受け取っても配列外参照しないよう上限で切り詰める
  size_t keyLength
      = std::min(static_cast<size_t>(request.keyLength), DecryptServer::MAX_KEY_LENGTH);
  size_t encryptedTextLength = std::min(static_cast<size_t>(request.encryptedTextLength),
                                        DecryptServer::MAX_ENCRYPTED_TEXT_LENGTH);
  // 復号回数は最低1回とし、際限なくサーバーを占有しないよう上限を設ける
  uint32_t iterations = std::min(std::max(request.iterations, 1u), MAX_ITERATIONS);

  std::string key(request.key, keyLength);
  std::string encryptedText(request.encryptedText, encryptedTextLength);
  std::string plainText;

  // 通信やシリアライズを含まない、純粋な復号演算時間のみを計測する
  Stopwatch stopwatch;
  for(uint32_t i = 0; i < iterations; i++) {
    plainText = Decrypter::decrypt(key, encryptedText);
  }
  response.decryptTimeMicro = stopwatch.elapsedMicro();

  response.iterations = iterations;
  response.success = !plainText.empty();

  size_t plainTextLength = std::min(plainText.size(), DecryptServer::MAX_PLAIN_TEXT_LENGTH);
  std::memcpy(response.plainText, plainText.data(), plainTextLength);
  response.plainTextLength = static_cast<uint16_t>(plainTextLength);

  if(!response.success) {
    Logger::error("DecryptActionHandler:execute: 復号に失敗しました");
  }
}
