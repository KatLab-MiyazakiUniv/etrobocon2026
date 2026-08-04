/**
 * @file   RemoteDecryptService.h
 * @brief  復号サーバーへ接続して復号を実行するクラス
 * @author HaruArima08
 */

#ifndef REMOTE_DECRYPT_SERVICE_H
#define REMOTE_DECRYPT_SERVICE_H

#include "IDecryptService.h"
#include "DecryptClient.h"

/**
 * @brief 復号サーバーへ接続して復号を実行するクラス
 * @note LOCAL_SOCKETとREMOTEは接続先が違うだけで処理は同一なので、本クラスで両方を扱う
 */
class RemoteDecryptService : public IDecryptService {
 public:
  /**
   * @brief コンストラクタ
   * @param _client 復号サーバーと通信するクライアント
   * @param _mode 復号モード（LOCAL_SOCKETまたはREMOTE）
   */
  RemoteDecryptService(DecryptClient& _client, DecryptMode _mode);

  /**
   * @brief 復号を実行し、所要時間を計測する
   * @param key 復号キー
   * @param encryptedText 暗号文
   * @param iterations 復号を繰り返す回数
   * @param measurement 計測結果の格納先
   * @return true 復号に成功した場合
   * @return false 復号または通信に失敗した場合
   */
  bool decrypt(const std::string& key, const std::string& encryptedText, uint32_t iterations,
               DecryptMeasurement& measurement) override;

  /**
   * @brief 通信のみの往復時間を計測する
   * @param roundTripMicro 往復時間の格納先(マイクロ秒)
   * @return true 計測に成功した場合
   * @return false 通信に失敗した場合
   */
  bool measurePing(uint64_t& roundTripMicro) override;

  /**
   * @brief 復号モードを取得する
   * @return DecryptMode 復号モード
   */
  DecryptMode getMode() const override;

 private:
  DecryptClient& client;  // 復号サーバーと通信するクライアントへの参照
  DecryptMode mode;       // 復号モード
};

#endif  // REMOTE_DECRYPT_SERVICE_H
