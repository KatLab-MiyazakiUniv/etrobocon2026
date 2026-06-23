/**
 * @file   Decrypter.cpp
 * @brief  XOR暗号の復号を行うクラス
 * @author okuyama0528
 */

#include "Decrypter.h"

Decrypter::Decrypter(const std::string& key) : key(key) {}

std::string Decrypter::decrypt(const std::string& input)
{
  std::string result = input;

  for(size_t i = 0; i < input.size(); i++) {
    result[i] = input[i] ^ key[i % key.size()];
  }

  return result;
}