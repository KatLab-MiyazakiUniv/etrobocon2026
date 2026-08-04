/**
 * @file   DecryptMode.h
 * @brief  復号処理の配置先を表す列挙型
 * @author HaruArima08
 */

#ifndef DECRYPT_MODE_H
#define DECRYPT_MODE_H

#include <cstdint>

/**
 * @brief 復号処理をどこで実行するかを表す
 * @note トレードオフ分析の案A(PCへ配置)がREMOTE、案B(ラズパイへ配置)がLOCAL_SOCKETに対応する
 */
enum class DecryptMode : uint8_t {
  LOCAL_INPROC = 0,  // 呼び出し元のプロセス内で復号する（通信なしの下限値を測るための基準）
  LOCAL_SOCKET = 1,  // ラズパイ上の復号サーバーへローカル接続して復号する（案B）
  REMOTE = 2         // PC上の復号サーバーへ無線接続して復号する（案A）
};

/**
 * @brief 復号モードを表す文字列を取得する
 * @param mode 復号モード
 * @return const char* 復号モードの名前
 */
inline const char* toDecryptModeName(DecryptMode mode)
{
  switch(mode) {
    case DecryptMode::LOCAL_INPROC:
      return "LOCAL_INPROC";
    case DecryptMode::LOCAL_SOCKET:
      return "LOCAL_SOCKET";
    case DecryptMode::REMOTE:
      return "REMOTE";
    default:
      return "UNKNOWN";
  }
}

#endif  // DECRYPT_MODE_H
