/**
 * @file   DecryptActionHandler.h
 * @brief  復号を実行し演算時間を計測するクラス
 * @author HaruArima08
 */

#ifndef DECRYPT_ACTION_HANDLER_H
#define DECRYPT_ACTION_HANDLER_H

#include "Decrypter.h"
#include "DecryptProtocol.h"
#include "Logger.h"

class DecryptActionHandler {
 public:
  /**
   * @brief コンストラクタ
   */
  DecryptActionHandler();

  /**
   * @brief 復号アクションを実行する
   * @param request クライアントからのリクエスト
   * @param response クライアントへのレスポンス
   * @note レスポンスのdecryptTimeMicroには通信を含まない純粋な復号演算時間のみを格納する
   */
  void execute(const DecryptServer::DecryptRequest& request,
               DecryptServer::DecryptResponse& response);

 private:
  static constexpr uint32_t MAX_ITERATIONS = 10000;  // 1リクエストで許容する復号回数の上限
};

#endif  // DECRYPT_ACTION_HANDLER_H
