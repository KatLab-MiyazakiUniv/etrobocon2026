/**
 * @file   LocalDecryptService.h
 * @brief  自プロセス内で復号を実行するクラス
 * @author HaruArima08
 */

#ifndef LOCAL_DECRYPT_SERVICE_H
#define LOCAL_DECRYPT_SERVICE_H

#include "IDecryptService.h"

/**
 * @brief 自プロセス内で復号を実行するクラス
 * @note 通信を一切行わないため、演算能力のみを測る基準値として使う
 */
class LocalDecryptService : public IDecryptService {
 public:
  /**
   * @brief コンストラクタ
   */
  LocalDecryptService();

  /**
   * @brief 復号を実行し、所要時間を計測する
   * @param key 復号キー
   * @param encryptedText 暗号文
   * @param iterations 復号を繰り返す回数
   * @param measurement 計測結果の格納先
   * @return true 復号に成功した場合
   * @return false 復号に失敗した場合
   */
  bool decrypt(const std::string& key, const std::string& encryptedText, uint32_t iterations,
               DecryptMeasurement& measurement) override;

  /**
   * @brief 通信のみの往復時間を計測する
   * @param roundTripMicro 往復時間の格納先(マイクロ秒)
   * @return true 常にtrue（通信を行わないため常に0を返す）
   */
  bool measurePing(uint64_t& roundTripMicro) override;

  /**
   * @brief 復号モードを取得する
   * @return DecryptMode 常にLOCAL_INPROC
   */
  DecryptMode getMode() const override;
};

#endif  // LOCAL_DECRYPT_SERVICE_H
