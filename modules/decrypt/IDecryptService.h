/**
 * @file   IDecryptService.h
 * @brief  復号処理の実行と計測を行うための基底クラス
 * @author HaruArima08
 */

#ifndef I_DECRYPT_SERVICE_H
#define I_DECRYPT_SERVICE_H

#include "DecryptMode.h"
#include <cstdint>
#include <string>

/**
 * @brief 1回の復号にかかった時間の内訳
 * @note commMicroは通信そのものに加え、シリアライズやサーバー内の分岐処理も含む
 */
struct DecryptMeasurement {
  bool success = false;             // 復号に成功したかどうか
  std::string plainText;            // 復号した平文
  uint64_t totalMicro = 0;          // 呼び出し側で計測した全体の時間(マイクロ秒)
  uint64_t serverDecryptMicro = 0;  // 復号を実行した側の純粋な演算時間(マイクロ秒)
  uint64_t commMicro = 0;           // totalMicroからserverDecryptMicroを引いた時間(マイクロ秒)
};

class IDecryptService {
 public:
  /**
   * @brief デストラクタ
   */
  virtual ~IDecryptService() = default;

  /**
   * @brief 復号を実行し、所要時間を計測する
   * @param key 復号キー
   * @param encryptedText 暗号文
   * @param iterations 復号を繰り返す回数
   * @param measurement 計測結果の格納先
   * @return true 復号に成功した場合
   * @return false 復号または通信に失敗した場合
   */
  virtual bool decrypt(const std::string& key, const std::string& encryptedText,
                       uint32_t iterations, DecryptMeasurement& measurement)
      = 0;

  /**
   * @brief 通信のみの往復時間を計測する
   * @param roundTripMicro 往復時間の格納先(マイクロ秒)
   * @return true 計測に成功した場合
   * @return false 通信に失敗した場合
   */
  virtual bool measurePing(uint64_t& roundTripMicro) = 0;

  /**
   * @brief 復号モードを取得する
   * @return DecryptMode 復号モード
   */
  virtual DecryptMode getMode() const = 0;
};

#endif  // I_DECRYPT_SERVICE_H
