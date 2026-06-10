/**
 * @file   DecodeTest.cpp
 * @brief  暗号文の複号処理クラスのテスト
 * @author yutaro-1214
 */

#include <gtest/gtest.h>
#include "Decode.h"

namespace etrobocon2026_test {
  // 正しく複号できるかのテスト
  TEST(DecodeTest, SuccesDecode)
  {
    std::string plainText = "Hello";  // 平文
    std::string key = "1234";         // 複号キー

    // 平文を暗号化した文
    std::string encryptedText = "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";

    EXPECT_EQ(plainText, Decode::decrypt(key, encryotedText));
  }

  // 違う鍵では復号できないかのテスト
  TEST(DecodeTest, IncorrectKey)
  {
    std::string plainText = "Hello";  // 平文
    std::string key = "1234";         // 複号キー

    // 平文を暗号化した文
    std::string encryptedText = "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";

    std::string notKey = "1111";  // 正しくないキー

    EXPECT_EQ("", Decode::decrypt(notKey, encryptedText));
  }
  // 違う暗号文では復号できないかのテスト
  TEST(DecodeTest, IncorrectCode)
  {
    std::string plainText = "Hello";  // 平文
    std::string key = "1234";         // 複号キー

    // 平文を暗号化した文
    std::string encryptedText = "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";

    // 正しくない暗号文
    std::string notEncryptedText = "1111U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";

    EXPECT_EQ("", Decode::decrypt(key, notEncryptedText));
  }

}  // namespace etrobocon2026_test