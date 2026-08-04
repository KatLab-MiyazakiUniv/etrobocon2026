/**
 * @file   RemoteDecryptServiceTest.cpp
 * @brief  RemoteDecryptServiceクラスをテストする
 * @author HaruArima08
 */

#include <gtest/gtest.h>
#include "RemoteDecryptService.h"
#include "MockByteStreamNetworkSystem.h"
#include <cstring>
#include <string>

namespace etrobocon2026_test {

  /**
   * @brief 任意のレスポンスを返す復号クライアントのスタブ
   */
  class StubDecryptClient : public DecryptClient {
   public:
    DecryptServer::DecryptResponse stubResponse;  // executeDecryptで返すレスポンス
    bool shouldSucceed = true;                    // 通信の成否を切り替えるフラグ
    DecryptServer::DecryptRequest lastRequest;    // 最後に受け取ったリクエスト

    /**
     * @brief コンストラクタ
     * @param _netSys 注入する具象クラス
     */
    explicit StubDecryptClient(INetworkSystem& _netSys) : DecryptClient(_netSys) {}

    /**
     * @brief 復号アクションのスタブ
     */
    bool executeDecrypt(const DecryptServer::DecryptRequest& request,
                        DecryptServer::DecryptResponse& response) override
    {
      lastRequest = request;
      if(!shouldSucceed) return false;
      response = stubResponse;
      return true;
    }

    /**
     * @brief Pingアクションのスタブ
     */
    bool executePing(const DecryptServer::PingRequest& request,
                     DecryptServer::PingResponse& response) override
    {
      if(!shouldSucceed) return false;
      response.success = true;
      return true;
    }
  };

  /**
   * @brief 復号成功を表すスタブレスポンスを組み立てる
   * @param plainText 返す平文
   * @param decryptTimeMicro 返す演算時間(マイクロ秒)
   * @return DecryptServer::DecryptResponse 組み立てたレスポンス
   */
  static DecryptServer::DecryptResponse makeStubResponse(const std::string& plainText,
                                                         uint64_t decryptTimeMicro)
  {
    DecryptServer::DecryptResponse response;
    response.success = true;
    response.decryptTimeMicro = decryptTimeMicro;
    response.plainTextLength = static_cast<uint16_t>(plainText.size());
    std::memcpy(response.plainText, plainText.data(), plainText.size());
    return response;
  }

  // 指定した復号モードを返すことを確認する
  TEST(RemoteDecryptServiceTest, GetMode)
  {
    MockByteStreamNetworkSystem mockNet;
    StubDecryptClient client(mockNet);

    RemoteDecryptService localSocketService(client, DecryptMode::LOCAL_SOCKET);
    RemoteDecryptService remoteService(client, DecryptMode::REMOTE);

    EXPECT_EQ(DecryptMode::LOCAL_SOCKET, localSocketService.getMode());
    EXPECT_EQ(DecryptMode::REMOTE, remoteService.getMode());
  }

  // レスポンスの平文が計測結果へ格納されることを確認する
  TEST(RemoteDecryptServiceTest, DecryptSuccess)
  {
    MockByteStreamNetworkSystem mockNet;
    StubDecryptClient client(mockNet);
    client.stubResponse = makeStubResponse("Hello", 1000);
    RemoteDecryptService service(client, DecryptMode::REMOTE);

    DecryptMeasurement measurement;
    EXPECT_TRUE(service.decrypt("1234", "dummy", 1, measurement));

    std::string expected = "Hello";
    EXPECT_TRUE(measurement.success);
    EXPECT_EQ(expected, measurement.plainText);
  }

  // 通信オーバヘッドが全体時間から演算時間を引いた値になることを確認する
  TEST(RemoteDecryptServiceTest, CommMicroIsTotalMinusServerDecrypt)
  {
    MockByteStreamNetworkSystem mockNet;
    StubDecryptClient client(mockNet);
    client.stubResponse = makeStubResponse("Hello", 0);  // 演算時間を0として通信分のみを見る
    RemoteDecryptService service(client, DecryptMode::REMOTE);

    DecryptMeasurement measurement;
    service.decrypt("1234", "dummy", 1, measurement);

    EXPECT_EQ(measurement.totalMicro - measurement.serverDecryptMicro, measurement.commMicro);
  }

  // 演算時間が全体時間を上回った場合でも、通信オーバヘッドが0に丸められることを確認する
  TEST(RemoteDecryptServiceTest, CommMicroIsClampedToZero)
  {
    MockByteStreamNetworkSystem mockNet;
    StubDecryptClient client(mockNet);
    // 全体時間より大きな演算時間を返させる
    client.stubResponse = makeStubResponse("Hello", 1000000000);
    RemoteDecryptService service(client, DecryptMode::REMOTE);

    DecryptMeasurement measurement;
    service.decrypt("1234", "dummy", 1, measurement);

    uint64_t expected = 0;
    EXPECT_EQ(expected, measurement.commMicro);
  }

  // リクエストへ復号キーと暗号文が正しく詰められることを確認する
  TEST(RemoteDecryptServiceTest, RequestIsBuiltCorrectly)
  {
    MockByteStreamNetworkSystem mockNet;
    StubDecryptClient client(mockNet);
    client.stubResponse = makeStubResponse("Hello", 100);
    RemoteDecryptService service(client, DecryptMode::REMOTE);

    DecryptMeasurement measurement;
    service.decrypt("1234", "abcdef", 7, measurement);

    uint8_t expectedKeyLength = 4;
    uint16_t expectedTextLength = 6;
    uint32_t expectedIterations = 7;
    EXPECT_EQ(expectedKeyLength, client.lastRequest.keyLength);
    EXPECT_EQ(expectedTextLength, client.lastRequest.encryptedTextLength);
    EXPECT_EQ(expectedIterations, client.lastRequest.iterations);
    EXPECT_EQ(std::string("1234"), std::string(client.lastRequest.key, 4));
  }

  // 通信に失敗した場合、falseを返すことを確認する
  TEST(RemoteDecryptServiceTest, DecryptFailsOnCommunicationError)
  {
    MockByteStreamNetworkSystem mockNet;
    StubDecryptClient client(mockNet);
    client.shouldSucceed = false;
    RemoteDecryptService service(client, DecryptMode::REMOTE);

    DecryptMeasurement measurement;
    EXPECT_FALSE(service.decrypt("1234", "dummy", 1, measurement));
    EXPECT_FALSE(measurement.success);
  }

  // 復号キーが最大長を超える場合、通信せずに失敗することを確認する
  TEST(RemoteDecryptServiceTest, DecryptFailsOnOversizedKey)
  {
    MockByteStreamNetworkSystem mockNet;
    StubDecryptClient client(mockNet);
    RemoteDecryptService service(client, DecryptMode::REMOTE);

    std::string oversizedKey(DecryptServer::MAX_KEY_LENGTH + 1, 'a');
    DecryptMeasurement measurement;
    EXPECT_FALSE(service.decrypt(oversizedKey, "dummy", 1, measurement));
  }

  // 暗号文が最大長を超える場合、通信せずに失敗することを確認する
  TEST(RemoteDecryptServiceTest, DecryptFailsOnOversizedEncryptedText)
  {
    MockByteStreamNetworkSystem mockNet;
    StubDecryptClient client(mockNet);
    RemoteDecryptService service(client, DecryptMode::REMOTE);

    std::string oversizedText(DecryptServer::MAX_ENCRYPTED_TEXT_LENGTH + 1, 'a');
    DecryptMeasurement measurement;
    EXPECT_FALSE(service.decrypt("1234", oversizedText, 1, measurement));
  }

  // Pingの往復時間を計測できることを確認する
  TEST(RemoteDecryptServiceTest, MeasurePingSucceeds)
  {
    MockByteStreamNetworkSystem mockNet;
    StubDecryptClient client(mockNet);
    RemoteDecryptService service(client, DecryptMode::REMOTE);

    uint64_t roundTripMicro = 0;
    EXPECT_TRUE(service.measurePing(roundTripMicro));
  }

  // 通信に失敗した場合、Pingの計測もfalseを返すことを確認する
  TEST(RemoteDecryptServiceTest, MeasurePingFailsOnCommunicationError)
  {
    MockByteStreamNetworkSystem mockNet;
    StubDecryptClient client(mockNet);
    client.shouldSucceed = false;
    RemoteDecryptService service(client, DecryptMode::REMOTE);

    uint64_t roundTripMicro = 0;
    EXPECT_FALSE(service.measurePing(roundTripMicro));
  }

}  // namespace etrobocon2026_test
