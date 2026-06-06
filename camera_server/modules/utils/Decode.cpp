/**
 * @file   Decode.cpp
 * @brief  ヒントカード2の複号処理クラス
 * @author yutaro-1214
 */

#include "Decode.h"

#include <openssl/evp.h>
#include <openssl/sha.h>

#include <cstring>
#include <stdexcept>
#include <vector>

std::string Decode::decrypt(const std::string& encryptedText)
{
  // Base64デコード
  std::vector<unsigned char> decoded(encryptedText.size());

  int decodedLength = EVP_DecodeBlock(decoded.data(),
                                      reinterpret_cast<const unsigned char*>(encryptedText.c_str()),
                                      encryptedText.length());

  if(decodedLength < 16) {
    throw std::runtime_error("Invalid encrypted text");
  }

  // Salted__確認
  if(std::memcmp(decoded.data(), "Salted__", 8) != 0) {
    throw std::runtime_error("Salt header not found");
  }

  // Salt取得
  unsigned char salt[8];
  std::memcpy(salt, decoded.data() + 8, 8);

  // AES-128用鍵生成
  unsigned char aesKey[16];

  if(!PKCS5_PBKDF2_HMAC(key.c_str(), key.length(), salt, sizeof(salt), 10000, EVP_sha256(),
                        sizeof(aesKey), aesKey)) {
    throw std::runtime_error("PBKDF2 failed");
  }

  // 暗号データ取得
  const unsigned char* cipherText = decoded.data() + 16;
  int cipherLength = decodedLength - 16;

  EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();

  if(ctx == nullptr) {
    throw std::runtime_error("Failed to create cipher context");
  }

  if(!EVP_DecryptInit_ex(ctx, EVP_aes_128_ecb(), nullptr, aesKey, nullptr)) {
    EVP_CIPHER_CTX_free(ctx);
    throw std::runtime_error("DecryptInit failed");
  }

  std::vector<unsigned char> plain(cipherLength + EVP_MAX_BLOCK_LENGTH);

  int outLen1 = 0;
  int outLen2 = 0;

  if(!EVP_DecryptUpdate(ctx, plain.data(), &outLen1, cipherText, cipherLength)) {
    EVP_CIPHER_CTX_free(ctx);
    throw std::runtime_error("DecryptUpdate failed");
  }

  if(!EVP_DecryptFinal_ex(ctx, plain.data() + outLen1, &outLen2)) {
    EVP_CIPHER_CTX_free(ctx);
    throw std::runtime_error("DecryptFinal failed");
  }

  EVP_CIPHER_CTX_free(ctx);

  return std::string(reinterpret_cast<char*>(plain.data()), outLen1 + outLen2);
}