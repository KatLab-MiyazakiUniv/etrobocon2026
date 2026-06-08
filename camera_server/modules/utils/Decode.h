/**
 * @file   Decode.h
 * @brief  暗号化文字列を復号化するクラス
 * @author yutaro-1214 HaruArima08
 */

#ifndef DECODE_H
#define DECODE_H

#include <openssl/evp.h>
#include <array>
#include <optional>
#include <string>
#include <cstring>
#include <vector>
#include "Logger.h"

class Decode {
 public:
  /**
   * @brief 暗号化文字列を復号化する
   * @param key 復号キー
   * @param encryptedText 暗号文
   * @return 復号化した文字列（失敗時は空文字列）
   */
  static std::string decrypt(const std::string& key, const std::string& encryptedText);

 private:
  Decode();  // インスタンス化禁止

  /**
   * @brief Base64エンコード文字列を復号化する
   * @param encoded Base64エンコード暗号文
   * @return Base64デコードしたバイト列
   */
  static std::optional<std::vector<unsigned char>> decodeBase64(const std::string& encoded);

  /**
   * @brief 復号キーを導出する
   * @param key 復号キー
   * @param salt Salt値
   * @return AES-128の復号キー
   */
  static std::optional<std::array<unsigned char, 16>> deriveKey(const std::string& key,
                                                                const unsigned char* salt);

  /**
   * @brief AES-128-ECBで復号化する
   * @param cipherText 復号対象の暗号文
   * @param cipherLength 暗号文のバイト長
   * @param aesKey 復号キー
   * @return 復号化した文字列（失敗時は空文字列）
   */
  static std::string decryptAES(const unsigned char* cipherText, int cipherLength,
                                const unsigned char* aesKey);
};

#endif  // DECODE_H
