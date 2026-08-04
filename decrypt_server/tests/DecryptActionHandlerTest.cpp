/**
 * @file   DecryptActionHandlerTest.cpp
 * @brief  DecryptActionHandlerクラスをテストする
 * @author HaruArima08
 */

#include <gtest/gtest.h>
#include "DecryptActionHandler.h"
#include <cstring>
#include <string>

namespace etrobocon2026_test {

  // 平文"Hello"を鍵"1234"で暗号化した文
  static const char* ENCRYPTED_TEXT = "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";

  /**
   * @brief 復号リクエストを組み立てる
   * @param key 復号キー
   * @param encryptedText 暗号文
   * @param iterations 復号を繰り返す回数
   * @return DecryptServer::DecryptRequest 組み立てたリクエスト
   */
  static DecryptServer::DecryptRequest makeRequest(const std::string& key,
                                                   const std::string& encryptedText,
                                                   uint32_t iterations = 1)
  {
    DecryptServer::DecryptRequest request;
    request.keyLength = static_cast<uint8_t>(key.size());
    request.encryptedTextLength = static_cast<uint16_t>(encryptedText.size());
    request.iterations = iterations;
    std::memcpy(request.key, key.data(), key.size());
    std::memcpy(request.encryptedText, encryptedText.data(), encryptedText.size());
    return request;
  }

  // 対応した鍵を渡した場合、正しく復号できることを確認する
  TEST(DecryptActionHandlerTest, DecryptSuccess)
  {
    DecryptActionHandler handler;
    DecryptServer::DecryptRequest request = makeRequest("1234", ENCRYPTED_TEXT);
    DecryptServer::DecryptResponse response;

    handler.execute(request, response);

    std::string expected = "Hello";
    EXPECT_TRUE(response.success);
    EXPECT_EQ(expected, std::string(response.plainText, response.plainTextLength));
  }

  // 対応していない鍵を渡した場合、失敗として扱われることを確認する
  TEST(DecryptActionHandlerTest, DecryptFailsWithIncorrectKey)
  {
    DecryptActionHandler handler;
    DecryptServer::DecryptRequest request = makeRequest("1111", ENCRYPTED_TEXT);
    DecryptServer::DecryptResponse response;

    handler.execute(request, response);

    uint16_t expected = 0;
    EXPECT_FALSE(response.success);
    EXPECT_EQ(expected, response.plainTextLength);
  }

  // 復号回数がレスポンスに反映されることを確認する
  TEST(DecryptActionHandlerTest, IterationsAreEchoedBack)
  {
    DecryptActionHandler handler;
    DecryptServer::DecryptRequest request = makeRequest("1234", ENCRYPTED_TEXT, 5);
    DecryptServer::DecryptResponse response;

    handler.execute(request, response);

    uint32_t expected = 5;
    EXPECT_EQ(expected, response.iterations);
  }

  // 復号回数に0を指定しても、最低1回は復号されることを確認する
  TEST(DecryptActionHandlerTest, IterationsAreClampedToAtLeastOne)
  {
    DecryptActionHandler handler;
    DecryptServer::DecryptRequest request = makeRequest("1234", ENCRYPTED_TEXT, 0);
    DecryptServer::DecryptResponse response;

    handler.execute(request, response);

    uint32_t expected = 1;
    EXPECT_EQ(expected, response.iterations);
    EXPECT_TRUE(response.success);
  }

  // 復号回数を増やすと演算時間も増えることを確認する
  TEST(DecryptActionHandlerTest, DecryptTimeIncreasesWithIterations)
  {
    DecryptActionHandler handler;
    DecryptServer::DecryptResponse singleResponse;
    DecryptServer::DecryptResponse multipleResponse;

    DecryptServer::DecryptRequest singleRequest = makeRequest("1234", ENCRYPTED_TEXT, 1);
    DecryptServer::DecryptRequest multipleRequest = makeRequest("1234", ENCRYPTED_TEXT, 10);
    handler.execute(singleRequest, singleResponse);
    handler.execute(multipleRequest, multipleResponse);

    EXPECT_GT(multipleResponse.decryptTimeMicro, singleResponse.decryptTimeMicro);
  }

  // 長さの指定が最大値を超えていても、配列外参照せず失敗として扱われることを確認する
  TEST(DecryptActionHandlerTest, OversizedLengthIsClamped)
  {
    DecryptActionHandler handler;
    DecryptServer::DecryptRequest request = makeRequest("1234", ENCRYPTED_TEXT);
    request.keyLength = 255;              // MAX_KEY_LENGTHを超える不正な値
    request.encryptedTextLength = 65535;  // MAX_ENCRYPTED_TEXT_LENGTHを超える不正な値
    DecryptServer::DecryptResponse response;

    handler.execute(request, response);

    EXPECT_FALSE(response.success);
  }

}  // namespace etrobocon2026_test
