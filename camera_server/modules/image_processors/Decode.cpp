/**
 * @file   Decode.cpp
 * @brief  暗号復号処理
 */

#include "Decode.h"

std::string Decode::decrypt(const std::string& key,
                            const std::string& encrypted)
{
  std::string result = encrypted;

  // 簡易的なXOR復号
  for(size_t i = 0; i < encrypted.size(); i++) {
    result[i] = encrypted[i] ^ key[i % key.size()];
  }

  return result;
}