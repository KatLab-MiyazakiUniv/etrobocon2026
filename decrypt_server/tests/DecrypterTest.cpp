/**
 * @file   DecrypterTest.cpp
 * @brief  暗号文の復号処理クラスのテスト
 * @author yutaro-1214
 */

#include <gtest/gtest.h>
#include "Decrypter.h"

namespace etrobocon2026_test {
  // 対応した暗号文と復号鍵が対応している場合、正しく復号できるかのテスト
  TEST(DecrypterTest, SuccesDecrypt)
  {
    std::string plainText = "Hello";  // 平文
    std::string key = "1234";         // 復号鍵

    // 平文を暗号化した文
    std::string encryptedText = "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";

    EXPECT_EQ(plainText, Decrypter::decrypt(key, encryptedText));
  }

  // 暗号文と復号鍵が対応していない場合、空文字を返すことを確認する
  TEST(DecrypterTest, IncorrectKey)
  {
    std::string plainText = "Hello";  // 平文

    // 平文を暗号化した文
    std::string encryptedText = "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";

    std::string notKey = "1111";  // 対応していない鍵

    EXPECT_EQ("", Decrypter::decrypt(notKey, encryptedText));
  }

  // 空文字を渡した場合、空文字を返すことを確認する
  TEST(DecrypterTest, EmptyEncryptedText)
  {
    std::string key = "1234";  // 復号鍵

    EXPECT_EQ("", Decrypter::decrypt(key, ""));
  }

  // Base64として不正な文字列を渡した場合、空文字を返すことを確認する
  TEST(DecrypterTest, InvalidBase64)
  {
    std::string key = "1234";  // 復号鍵

    std::string invalidText = "!!!not_base64!!!";  // Base64デコードできない文字列

    EXPECT_EQ("", Decrypter::decrypt(key, invalidText));
  }

  // Saltヘッダを持たない暗号文を渡した場合、空文字を返すことを確認する
  TEST(DecrypterTest, MissingSaltHeader)
  {
    std::string key = "1234";  // 復号鍵

    // "0123456789abcdef0123456789abcdef"をBase64エンコードした文字列（Saltヘッダを含まない）
    std::string noSaltText = "MDEyMzQ1Njc4OWFiY2RlZjAxMjM0NTY3ODlhYmNkZWY=";

    EXPECT_EQ("", Decrypter::decrypt(key, noSaltText));
  }

}  // namespace etrobocon2026_test