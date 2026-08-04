/**
 * @file   DecryptProtocolTest.cpp
 * @brief  復号システムの通信プロトコル定義をテストする
 * @author HaruArima08
 */

#include <gtest/gtest.h>
#include "DecryptProtocol.h"
#include <cstddef>

namespace etrobocon2026_test {

  // コマンドが1バイトで表現されることを確認する
  TEST(DecryptProtocolTest, CommandSize)
  {
    size_t expected = 1;
    EXPECT_EQ(expected, DecryptServer::COMMAND_SIZE);
  }

  // 復号リクエストのバイトサイズが想定通りであることを確認する
  TEST(DecryptProtocolTest, DecryptRequestSize)
  {
    size_t expected = 296;
    EXPECT_EQ(expected, sizeof(DecryptServer::DecryptRequest));
  }

  // 復号レスポンスのバイトサイズが想定通りであることを確認する
  TEST(DecryptProtocolTest, DecryptResponseSize)
  {
    size_t expected = 272;
    EXPECT_EQ(expected, sizeof(DecryptServer::DecryptResponse));
  }

  // コマンドが必ず構造体の先頭に配置されることを確認する
  TEST(DecryptProtocolTest, CommandIsPlacedAtHeadOfRequest)
  {
    size_t expected = 0;
    EXPECT_EQ(expected, offsetof(DecryptServer::DecryptRequest, command));
    EXPECT_EQ(expected, offsetof(DecryptServer::PingRequest, command));
  }

  // 復号リクエストの各メンバがパディングを挟まず整列していることを確認する
  TEST(DecryptProtocolTest, DecryptRequestMemberOffsets)
  {
    EXPECT_EQ(static_cast<size_t>(1), offsetof(DecryptServer::DecryptRequest, keyLength));
    EXPECT_EQ(static_cast<size_t>(2), offsetof(DecryptServer::DecryptRequest, encryptedTextLength));
    EXPECT_EQ(static_cast<size_t>(4), offsetof(DecryptServer::DecryptRequest, iterations));
    EXPECT_EQ(static_cast<size_t>(8), offsetof(DecryptServer::DecryptRequest, key));
    EXPECT_EQ(static_cast<size_t>(40), offsetof(DecryptServer::DecryptRequest, encryptedText));
  }

  // 復号レスポンスの各メンバがパディングを挟まず整列していることを確認する
  TEST(DecryptProtocolTest, DecryptResponseMemberOffsets)
  {
    EXPECT_EQ(static_cast<size_t>(0), offsetof(DecryptServer::DecryptResponse, decryptTimeMicro));
    EXPECT_EQ(static_cast<size_t>(8), offsetof(DecryptServer::DecryptResponse, iterations));
    EXPECT_EQ(static_cast<size_t>(12), offsetof(DecryptServer::DecryptResponse, plainTextLength));
    EXPECT_EQ(static_cast<size_t>(14), offsetof(DecryptServer::DecryptResponse, success));
    EXPECT_EQ(static_cast<size_t>(16), offsetof(DecryptServer::DecryptResponse, plainText));
  }

  // 撮影システムとポート番号が衝突しないことを確認する
  TEST(DecryptProtocolTest, PortDoesNotConflictWithCameraServer)
  {
    int expected = 27016;
    EXPECT_EQ(expected, DecryptServer::DEFAULT_PORT);
  }

}  // namespace etrobocon2026_test
