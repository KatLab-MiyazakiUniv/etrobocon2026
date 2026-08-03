/**
 * @file MultiThread.h
 * @brief メンバ関数をマルチスレッドで実行するラッパークラス
 * @author sadomiya-sousi
 */

#include <thread>
#include <iostream>
#include <thread>
#include <functional>
#include <utility>
#include <chrono>
#include "Logger.h"

class MultiThread {
 public:
  /**
   * @brief 関数を別スレッドで実行するラッパー。関数呼び出しに0~1msかかる。
   * @param Func 関数オブジェクト
   * @param Args 関数オブジェクトを実行するための引数
   * @example
   *  静的メンバ関数の場合:
   *   MultiThread::wrap([=]() mutable { FrameSave::save(frame, filePath, fileName); });
   *  メンバ関数の場合:
   *   MultiThread::wrap([=]() mutable { インスタンス.メンバ関数(引数1, 引数2, 引数3,...); });
   */

  template <typename Func, typename... Args>
  static void wrap(Func&& func, Args&&... args)
  {
    Logger::info("MultiThread:開始");
    int start = ClockUtil::now();
    // 関数を定義
    auto task = [f = std::forward<Func>(func),
                 t = std::make_tuple(std::forward<Args>(args)...)]() mutable {
      // 別スレッド作成, 実行に失敗した場合に走行プログラムを停止させないようにtry{}catch{}
      try {
        std::apply(f, std::move(t));
      } catch(const std::exception& e) {
        Logger::error("MultiThread:エラー発生");
      }
    };

    int endTomakeFunc = ClockUtil::now();
    Logger::printfLog(Logger::INFO, "MultiThread: 関数作成にかかった時間は %d ms",
                      endTomakeFunc - start);
    // 関数を実行
    std::thread(std::move(task)).detach();
  }

 private:
  /**
   * @brief インスタンス化禁止
   */
  MultiThread();
};