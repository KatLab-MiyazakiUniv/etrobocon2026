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
    EXPECT_EQ("Hello", Decode::decrypt("1234", "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I="));
  }

  // 違う鍵では復号できないかのテスト
  TEST(DecodeTest, IncorrectKey)
  {
    EXPECT_EQ("", Decode::decrypt("1111", "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I="));
  }
  // 違う暗号文では復号できないかのテスト
  TEST(DecodeTest, IncorrectCode)
  {
    EXPECT_EQ("", Decode::decrypt("1234", "1111U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I="));
  }

}  // namespace etrobocon2026_test