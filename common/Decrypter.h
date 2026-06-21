/**
 * @file   Decrypter.cpp
 * @brief  XOR暗号の復号を行うクラス
 * @author okuyama0528
 */
#ifndef DECRYPTER_H
#define DECRYPTER_H

#include <string>

class Decrypter {
 public:
  /**
   * @brief コンストラクタ
   * @param key 復号キー（4文字）
   */
  explicit Decrypter(const std::string& key);

  /**
   * @brief 復号処理
   * @param input 暗号化された文字列
   * @return 復号後の文字列
   */
  std::string decrypt(const std::string& input);

 private:
  std::string key;  // 復号キー
};

#endif