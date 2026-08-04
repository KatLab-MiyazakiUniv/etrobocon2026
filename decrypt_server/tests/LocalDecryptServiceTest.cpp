/**
 * @file   LocalDecryptServiceTest.cpp
 * @brief  LocalDecryptServiceクラスをテストする
 * @author HaruArima08
 */

#include <gtest/gtest.h>
#include "LocalDecryptService.h"
#include <string>

namespace etrobocon2026_test {

  // 平文"Hello"を鍵"1234"で暗号化した文
  static const char* LOCAL_TEST_ENCRYPTED_TEXT = "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";

  // 復号モードがLOCAL_INPROCであることを確認する
  TEST(LocalDecryptServiceTest, GetMode)
  {
    LocalDecryptService service;
    EXPECT_EQ(DecryptMode::LOCAL_INPROC, service.getMode());
  }

  // 正しく復号できることを確認する
  TEST(LocalDecryptServiceTest, DecryptSuccess)
  {
    LocalDecryptService service;
    DecryptMeasurement measurement;

    EXPECT_TRUE(service.decrypt("1234", LOCAL_TEST_ENCRYPTED_TEXT, 1, measurement));

    std::string expected = "Hello";
    EXPECT_TRUE(measurement.success);
    EXPECT_EQ(expected, measurement.plainText);
  }

  // 通信を行わないため、通信オーバヘッドが0になることを確認する
  TEST(LocalDecryptServiceTest, CommMicroIsZero)
  {
    LocalDecryptService service;
    DecryptMeasurement measurement;

    service.decrypt("1234", LOCAL_TEST_ENCRYPTED_TEXT, 1, measurement);

    uint64_t expected = 0;
    EXPECT_EQ(expected, measurement.commMicro);
    EXPECT_EQ(measurement.totalMicro, measurement.serverDecryptMicro);
  }

  // 対応していない鍵を渡した場合、失敗として扱われることを確認する
  TEST(LocalDecryptServiceTest, DecryptFailsWithIncorrectKey)
  {
    LocalDecryptService service;
    DecryptMeasurement measurement;

    EXPECT_FALSE(service.decrypt("1111", LOCAL_TEST_ENCRYPTED_TEXT, 1, measurement));
    EXPECT_FALSE(measurement.success);
  }

  // 通信を行わないため、Pingの往復時間が0になることを確認する
  TEST(LocalDecryptServiceTest, MeasurePingReturnsZero)
  {
    LocalDecryptService service;
    uint64_t roundTripMicro = 999;

    EXPECT_TRUE(service.measurePing(roundTripMicro));

    uint64_t expected = 0;
    EXPECT_EQ(expected, roundTripMicro);
  }

  // 復号回数を増やすと演算時間も増えることを確認する
  TEST(LocalDecryptServiceTest, TotalMicroIncreasesWithIterations)
  {
    LocalDecryptService service;
    DecryptMeasurement single;
    DecryptMeasurement multiple;

    service.decrypt("1234", LOCAL_TEST_ENCRYPTED_TEXT, 1, single);
    service.decrypt("1234", LOCAL_TEST_ENCRYPTED_TEXT, 10, multiple);

    EXPECT_GT(multiple.totalMicro, single.totalMicro);
  }

}  // namespace etrobocon2026_test
