/**
 * @file   Utils.h
 * @brief  便利関数のクラス
 * @author sadomiya-sousi
 */

#include <string>
#include <chrono>
#include <iostream>
#include <ctime>

class Utils {
 public:
  /**
   * @brief 値をint型にキャストして文字列に変換するヘルパー関数
   * @param value 変換する値
   * @return std::string 変換後の文字列
   */
  static std::string toStrInt(double value);

  /**
   * @brief日本時間をスレッドセーフに取得
   * @return 時間をメンバ変数にもつ構造体
   *.tm_mon
   *.tm_mday
   *.tm_hour
   *.tm_min
   *.tm_sec
   */
  static std::tm getLocalTime();

 private:
  Utils();  // インスタンス化禁止
};
