/**
 * @file   DecodeTest.cpp
 * @brief  暗号文の複号処理クラスのテスト
 * @author yutaro-1214
 */

#include <gtest/gtest.h>
#include "Decode.h"

namespace etrobocon2026_test {
  // 対応した暗号文と複号鍵が対応している場合、正しく複号できるかのテスト
  TEST(DecodeTest, SuccesDecode)
  {
    std::string plainText = "Hello";  // 平文
    std::string key = "1234";         // 複号鍵

    // 平文を暗号化した文
    std::string encryptedText = "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";

    EXPECT_EQ(plainText, Decode::decrypt(key, encryptedText));
  }

  // 暗号文と複合鍵が対応していない場合、空文字を返すことを確認する
  TEST(DecodeTest, IncorrectKey)
  {
    std::string plainText = "Hello";  // 平文

    // 平文を暗号化した文
    std::string encryptedText = "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";

    std::string notKey = "1111";  // 対応していない鍵

    EXPECT_EQ("", Decode::decrypt(notKey, encryptedText));
  }

}  // namespace etrobocon2026_test