/**
 * @file   AesDecryptor.h
 * @brief  AES-128(ECB)による復号処理クラス
 * @author migaku2645
 */

#ifndef AES_DECRYPTOR_H
#define AES_DECRYPTOR_H

#include <string>
#include <vector>
#include <cstring>
#include <openssl/bio.h>
#include <openssl/evp.h>
#include "Logger.h"
#include <cctype>

class AesDecryptor {
 public:
  /**
   * @brief AES復号処理クラスのコンストラクタ
   * @param _key 4桁の復号キー
   * @param _ciphertext Base64形式の暗号文
   */
  AesDecryptor(const std::string& _key, const std::string& _ciphertext);

  /**
   * @brief デストラクタ
   */
  ~AesDecryptor();

  /**
   * @brief 暗号文をAES-128(ECB)で復号する
   * @return 復号された文字列
   */
  std::string decrypt();

 private:
  std::string key;
  std::string ciphertext;

  /**
   * @brief 4桁のキーを16バイトのAES鍵に変換する
   * @param aesKey 生成した16バイトのAES鍵
   */
  void createKey(unsigned char aesKey[16]);

  /**
   * @brief Base64形式の文字列をデコードする
   * @param input Base64形式の文字列
   * @return デコードされたバイナリデータ
   */
  std::vector<unsigned char> decodeBase64(const std::string& input);

  /**
   * @brief 復号キーが4桁の数字か確認する
   * @return 正しいキーの場合true
   */
  bool validateKey();
};

#endif