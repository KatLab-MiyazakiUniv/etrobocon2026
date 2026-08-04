/**
 * @file   DecryptProtocol.h
 * @brief  復号システムのSocket通信の共通定義ファイル
 * @author HaruArima08
 */

#ifndef DECRYPT_PROTOCOL_H
#define DECRYPT_PROTOCOL_H

#include <cstdint>
#include <cstddef>

namespace DecryptServer {

  static constexpr int DEFAULT_PORT = 27016;  // 復号サーバーのデフォルトのポート番号

  // ラズパイ内で復号する場合の接続先IPアドレス
  static constexpr const char* LOCAL_SERVER_IP = "127.0.0.1";

  static constexpr size_t MAX_KEY_LENGTH = 32;              // 復号キーの最大バイト長
  static constexpr size_t MAX_ENCRYPTED_TEXT_LENGTH = 256;  // 暗号文の最大バイト長
  static constexpr size_t MAX_PLAIN_TEXT_LENGTH = 256;      // 平文の最大バイト長

  /**
   * @brief サーバー側で実行可能なコマンド
   */
  enum class Command : uint8_t {
    DECRYPT = 0,       // 復号
    PING = 1,          // 復号を行わず即応答する（通信遅延のベースライン計測用）
    DISCONNECT = 254,  // サーバーから切断
    SHUTDOWN = 255     // サーバーをシャットダウン
  };

  constexpr size_t COMMAND_SIZE = sizeof(Command);  // コマンド型のバイトサイズ

  /**
   * @brief 復号リクエストのデータ構造
   * @note 先頭1バイトを必ずCommandにし，残りのメンバは自然整列するよう大きい型から順に宣言する
   */
  struct DecryptRequest {
    Command command = Command::DECRYPT;                     // DECRYPTを期待
    uint8_t keyLength = 0;                                  // keyの有効バイト数
    uint16_t encryptedTextLength = 0;                       // encryptedTextの有効バイト数
    uint32_t iterations = 1;                                // 復号を繰り返す回数
    char key[MAX_KEY_LENGTH] = { 0 };                       // 復号キー
    char encryptedText[MAX_ENCRYPTED_TEXT_LENGTH] = { 0 };  // 暗号文
  };

  /**
   * @brief 復号レスポンスのデータ構造
   */
  struct DecryptResponse {
    uint64_t decryptTimeMicro = 0;                  // サーバー側の純粋な復号演算時間の合計[μs]
    uint32_t iterations = 0;                        // 実際に復号を実行した回数
    uint16_t plainTextLength = 0;                   // plainTextの有効バイト数
    bool success = false;                           // 復号に成功したかどうか
    uint8_t reserved = 0;                           // 整列を明示するための予約領域
    char plainText[MAX_PLAIN_TEXT_LENGTH] = { 0 };  // 復号した平文
  };

  /**
   * @brief 通信遅延のベースラインを計測するためのリクエストのデータ構造
   */
  struct PingRequest {
    Command command = Command::PING;  // PINGを期待
    uint8_t reserved[7] = { 0 };      // 整列を明示するための予約領域
  };

  /**
   * @brief 通信遅延のベースラインを計測するためのレスポンスのデータ構造
   */
  struct PingResponse {
    bool success = true;          // 常にtrue
    uint8_t reserved[7] = { 0 };  // 整列を明示するための予約領域
  };

  constexpr size_t DECRYPT_REQUEST_SIZE = sizeof(DecryptRequest);    // リクエストのバイトサイズ
  constexpr size_t DECRYPT_RESPONSE_SIZE = sizeof(DecryptResponse);  // レスポンスのバイトサイズ

  // ラズパイ(aarch64)とPC(x86_64)で構造体レイアウトが一致することを保証する
  // 両者ともリトルエンディアンのLP64環境であることを前提とし，バイトオーダ変換は行わない
  static_assert(DECRYPT_REQUEST_SIZE == 296, "DecryptRequestのレイアウトが想定と異なります");
  static_assert(DECRYPT_RESPONSE_SIZE == 272, "DecryptResponseのレイアウトが想定と異なります");
  static_assert(sizeof(PingRequest) == 8, "PingRequestのレイアウトが想定と異なります");
  static_assert(sizeof(PingResponse) == 8, "PingResponseのレイアウトが想定と異なります");
}  // namespace DecryptServer
#endif  // DECRYPT_PROTOCOL_H
