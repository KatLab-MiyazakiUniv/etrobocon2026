/**
 * @file   Decode.h
 * @brief  ヒントカード2の複号処理の親クラス
 * @author yutaro-1214
 */
#ifndef DECODE_H
#define DECODE_H

#include <string>

class Decode {
 public:
    /**
   * @brief 復号キーを設定
   * @param key 復号キー
   */
  explicit Decode(const std::string& key);

  /**
   * @brief 暗号文を復号する
   * @param encryptedText Base64形式の暗号文
   * @return 復号結果
   */
  std::string decrypt(const std::string& encryptedText);

 private:
  std::string key;
};

#endif