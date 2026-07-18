/**
 * @file MultiThreadTest.cpp
 * @brief MultiThreadクラスのテスト
 * @author sadomiya-sousi
 */

#include <gtest/gtest.h>
#include <chrono>
#include <atomic>
#include <stdexcept>

#include "MultiThread.h"
#include "Logger.h"

namespace etrobocon2026_test {

  TEST(MultiThreadTest, WrapFrameSave) {}

  // 引数なしのラムダ関数を実行
  TEST(MultiThreadTest, WrapNoArgument)
  {
    std::atomic<bool> called = false;
    MultiThread::wrap([&called]() {
      // ラムダ関数の処理実行時に7ms待機
      Logger::debug("WrapNoArgument:called = trueを待機");
      std::this_thread::sleep_for(std::chrono::milliseconds(7));
      called = true;
      Logger::debug("WrapNoArgument:called = true実行しました");
    });

    int i = 0;
    for(i = 0; i < 100 && !called; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Logger::printfLog(Logger::INFO, "%dカウントで終了", i);
    EXPECT_TRUE(called);
  }

  // 参照型の引数を持つラムダ関数を実行
  TEST(MultiThreadTest, WrapWithArgumentsTest)
  {
    std::atomic<int> result = 0;
    int value = 42;
    int expectedValue = 47;
    std::string text = "hello";
    // 参照渡しをするために std::ref を使用する必要あり
    MultiThread::wrap(
        [](int val, std::string str, std::atomic<int>& res) { res = val + str.length(); }, value,
        text, std::ref(result));
    for(int i = 0; i < 20 && result == 0; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_EQ(result, expectedValue);
  }

  // 別スレッドに投げた処理よりも先に, 元スレッドの処理が次の行に進むことを検証
  TEST(MultiThreadTest, WrapAsynchronousNonBlockingTest)
  {
    std::atomic<bool> done = false;
    int startTime = ClockUtil::now();
    // 別スレッドで元スレッドのdoneをtrueに書き換える
    MultiThread::wrap([&done]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
      done = true;
    });

    int endTime = ClockUtil::now();
    int elapsed = endTime - startTime;
    // MultiThread::wrap後
    // 元スレッドに戻るため、経過時間は別スレッド中の停止時間の4msより短い
    EXPECT_LT(elapsed, 3);
    // 呼び出し直後なので、スレッド内の処理は完了していないはず
    EXPECT_FALSE(done);
    // 5ms後に別スレッドが done = true を書き換えていることを検証
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_TRUE(done);
  }

  // 別スレッドでエラーが起きても、元スレッドが停止しないことを検証
  TEST(MultiThreadTest, WrapExceptionSafetyTest)
  {
    std::atomic<bool> finished = false;

    MultiThread::wrap([&finished]() {
      finished = true;
      throw std::runtime_error("Test exception in thread");
    });

    // 実行完了を待つ
    for(int i = 0; i < 100 && !finished; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // 別スレッドは最後で例外によって中断するが、
    // wrap内の try-catch によりプロセス全体は停止しない
    EXPECT_TRUE(finished);
  }

  // 元スレッドと別スレッドのスレッドIDが異なることを検証
  TEST(MultiThreadTest, WrapFailAssumedSameThread)
  {
    std::thread::id main_thread_id = std::this_thread::get_id();
    std::thread::id child_thread_id;
    std::atomic<bool> done = false;

    MultiThread::wrap([&child_thread_id, &done]() {
      child_thread_id = std::this_thread::get_id();
      done = true;
    });

    // スレッドの実行完了を待つ
    for(int i = 0; i < 10 && !done; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    EXPECT_TRUE(done);
    // 別スレッドで実行されるため、メインスレッドのIDとは異なる。
    EXPECT_NE(main_thread_id, child_thread_id);  // 意図的に失敗するアサーション
  }

  // ダミークラスの定義
  class DummyClassB {
   public:
    std::atomic<int> value = 0;
  };

  class DummyClassA {
   public:
    void memberFunc(DummyClassB& b) { b.value = 100; }
  };

  // 7. 他のクラスの参照を引数に持つメンバ関数を非同期で実行するテスト
  TEST(MultiThreadTest, WrapMemberFunctionWithRefArgTest)
  {
    DummyClassA objA;
    DummyClassB objB;

    // メンバ関数ポインタ、対象オブジェクトポインタ、引数（参照）を渡して非同期実行

    MultiThread::wrap(&DummyClassA::memberFunc, &objA, std::ref(objB));
    // MultiThread::wrap(objA.memberFunc, std::ref(objB));

    // 実行完了を待つ (最大1秒)
    for(int i = 0; i < 100 && objB.value == 0; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 値が 100 に書き換わっていることを確認
    EXPECT_EQ(objB.value, 100);
  }

}  // namespace etrobocon2026_test