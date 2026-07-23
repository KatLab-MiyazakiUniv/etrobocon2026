/**
 * @file   Decode.h
 * @brief  暗号化された文字列の復号処理
 */

#ifndef DECODE_H
#define DECODE_H

#include <string>

class Decode
{
public:
  /**
   * @brief 暗号文を復号する
   * @param key 復号キー
   * @param encrypted 暗号化文字列
   * @return 復号後文字列
   */
  static std::string decrypt(const std::string& key,
                             const std::string& encrypted);
};

#endif